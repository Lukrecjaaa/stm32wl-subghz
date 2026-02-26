use defmt::{debug, trace};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

use crate::{
    RadioError,
    radio::{PaSelection, PacketType, Radio, RampTime, RxGain, irq},
    traits::{Configure, Receive, Transmit},
};

// Re-export shared FSK types
pub use super::fsk::{
    AddrComp, Bandwidth, Bitrate, CrcType, PacketLengthType, PreambleDetLength, PulseShape,
};

impl Bitrate {
    /// Get the 3-byte Fdev register for MSK (Fdev = bitrate / 4)
    /// Register = deviation_hz * 2^25 / 32 MHz
    fn fdev_bytes(self) -> [u8; 3] {
        let deviation_hz = self.bps() / 4;
        let val = ((deviation_hz as u64) * (1 << 25)) / 32_000_000;
        [(val >> 16) as u8, (val >> 8) as u8, val as u8]
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub struct MskConfig {
    pub frequency: u32,
    pub bitrate: Bitrate,
    pub pulse_shape: PulseShape,
    /// Bandwidth of the rx side
    /// Should be >= 1.5 * bitrate for MSK
    /// (Carson's rule: BW = 2 * (Fdev + bitrate/2), with Fdev = bitrate/4)
    /// So for 10kbps signal, it should be
    /// 2 * (10000/4 + 10000/2) = 15000 Hz
    pub bandwidth: Bandwidth,
    pub preamble_len: u16,
    pub preamble_det: PreambleDetLength,
    /// Sync word bytes (1-8) written to SUBGHZ_GSYNCR (0x06C0)
    pub sync_word: [u8; 8],
    pub addr_comp: AddrComp,
    pub packet_type: PacketLengthType,
    pub crc: CrcType,
    pub whitening: bool,
    pub rx_gain: RxGain,
    pub pa: PaSelection,
    pub power_dbm: i8,
    pub ramp: RampTime,
}

impl Default for MskConfig {
    fn default() -> Self {
        Self {
            frequency: 868_100_000,
            bitrate: Bitrate::Custom(600),
            pulse_shape: PulseShape::GaussianBt05,
            bandwidth: Bandwidth::Bw4_8kHz,
            preamble_len: 32,
            preamble_det: PreambleDetLength::Bits8,
            // default values taken from RF0461 reference manual
            sync_word: [0x97, 0x23, 0x52, 0x25, 0x56, 0x53, 0x65, 0x64],
            addr_comp: AddrComp::Off,
            packet_type: PacketLengthType::Fixed,
            crc: CrcType::Crc2Byte,
            whitening: true,
            rx_gain: RxGain::Boosted,
            pa: PaSelection::LowPower,
            power_dbm: 14,
            ramp: RampTime::Us40,
        }
    }
}

/// (G)MSK modulation implemented via FSK with modulation index 0.5
/// Borrows a Radio, implements Configure + Transmit + Receive
pub struct MskRadio<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> {
    radio: &'a mut Radio<SPI, TX, RX, EN>,
    payload_len: u8,
    config: MskConfig,
}

impl<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin>
    MskRadio<'a, SPI, TX, RX, EN>
{
    pub fn new(radio: &'a mut Radio<SPI, TX, RX, EN>) -> Self {
        Self {
            radio,
            payload_len: 0,
            config: MskConfig::default(),
        }
    }

    /// Re-send SetPacketParams with updated payload length
    async fn update_payload_len(&mut self, len: u8) -> Result<(), RadioError> {
        debug!("Updating payload length to {}", len);
        if len == self.payload_len {
            return Ok(());
        }
        self.payload_len = len;
        self.send_packet_params(len).await
    }

    /// Send FSK SetPacketParams with the given payload length
    async fn send_packet_params(&mut self, payload_len: u8) -> Result<(), RadioError> {
        let cfg = &self.config;
        self.radio
            .set_packet_params(&[
                (cfg.preamble_len >> 8) as u8,
                cfg.preamble_len as u8,
                cfg.preamble_det as u8,
                cfg.sync_word.len() as u8 * 8, // SyncWordLen in bits
                cfg.addr_comp as u8,
                cfg.packet_type as u8,
                payload_len,
                cfg.crc as u8,
                if cfg.whitening { 0x01 } else { 0x00 },
            ])
            .await
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Configure
    for MskRadio<'_, SPI, TX, RX, EN>
{
    type Config = MskConfig;

    async fn configure(&mut self, config: &Self::Config) -> Result<(), RadioError> {
        self.config = *config;

        // Use FSK packet type - MSK is FSK with modulation index 0.5
        self.radio.set_packet_type(PacketType::Fsk).await?;

        // Write sync word to SUBGHZ_GSYNCR (0x06C0)
        self.radio.write_register(0x06C0, &config.sync_word).await?;

        // Set FSK packet params
        self.send_packet_params(0).await?;

        // RF frequency
        self.radio.set_rf_frequency(config.frequency).await?;

        // Modulation params: FSK format with Fdev = bitrate/4 for MSK
        let br = config.bitrate.to_bytes();
        let fdev = config.bitrate.fdev_bytes();
        self.radio
            .set_modulation_params(&[
                br[0],
                br[1],
                br[2],
                config.pulse_shape as u8,
                config.bandwidth as u8,
                fdev[0],
                fdev[1],
                fdev[2],
            ])
            .await?;

        // PA config + TX power
        self.radio
            .set_output_power(config.pa, config.power_dbm, config.ramp)
            .await?;

        Ok(())
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Transmit
    for MskRadio<'_, SPI, TX, RX, EN>
{
    async fn tx(&mut self, data: &[u8]) -> Result<(), RadioError> {
        if data.len() > 255 {
            return Err(RadioError::PayloadTooLarge);
        }

        // Write payload to radio buffer
        self.radio.set_buffer_base(0x00, 0x00).await?;
        self.radio.write_buffer(0x00, data).await?;

        // Update packet params with actual payload length
        self.update_payload_len(data.len() as u8).await?;

        // Clear any stale IRQ flags before starting TX
        self.radio.clear_irq(irq::ALL).await?;

        // Enable IRQs on DIO1
        self.radio.set_dio1_irq(irq::TX_DONE | irq::TIMEOUT).await?;

        // Start TX
        self.radio.set_tx(0).await?;

        // Wait until it's done or until timeout
        let status = self.radio.poll_irq(irq::TX_DONE | irq::TIMEOUT).await?;
        if status & irq::TIMEOUT != 0 {
            return Err(RadioError::Timeout);
        }

        Ok(())
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Receive
    for MskRadio<'_, SPI, TX, RX, EN>
{
    async fn rx(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, RadioError> {
        // Set max payload length we can accept
        let max_len = buf.len().min(255) as u8;
        self.update_payload_len(max_len).await?;

        // Set buffer base addresses
        self.radio.set_buffer_base(0x00, 0x00).await?;

        // Clear any stale IRQ flags before starting RX
        self.radio.clear_irq(irq::ALL).await?;

        // Enable RX-related IRQs on DIO1
        self.radio
            .set_dio1_irq(irq::RX_DONE | irq::TIMEOUT | irq::CRC_ERR | irq::SYNC_WORD_VALID)
            .await?;

        // Stop RX timer on preamble detection (required for proper RX behavior)
        self.radio.set_stop_rx_timer_on_preamble(true).await?;

        // Set RX gain
        self.radio
            .write_register(0x08AC, &[self.config.rx_gain as u8])
            .await?;

        // Convert ms to 15.625µs steps, 0 = single, 0xFFFFFF = continuous
        let timeout_steps = if timeout_ms == 0 {
            0
        } else {
            timeout_ms.saturating_mul(64).min(0xFFFFFF)
        };
        self.radio.set_rx(timeout_steps).await?;

        // Wait for something to happen
        let status = self
            .radio
            .poll_irq(irq::RX_DONE | irq::TIMEOUT | irq::CRC_ERR)
            .await?;

        // Check what happened
        if status & irq::TIMEOUT != 0 {
            return Err(RadioError::Timeout);
        }
        if status & irq::CRC_ERR != 0 {
            return Err(RadioError::CrcInvalid);
        }

        // Read received data from the radio buffer
        let (len, offset) = self.radio.get_rx_buffer_status().await?;
        let read_len = len.min(buf.len() as u8);
        self.radio
            .read_buffer(offset, &mut buf[..read_len as usize])
            .await?;

        trace!("Got data {:x}", &mut buf[..read_len as usize]);

        Ok(read_len as usize)
    }
}
