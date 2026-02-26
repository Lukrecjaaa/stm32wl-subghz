use defmt::{debug, trace};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

use crate::{
    RadioError,
    radio::{PaSelection, PacketType, Radio, RampTime, RxGain, irq},
    traits::{Configure, Receive, Transmit},
};

/// (G)FSK bitrate
/// Formula: register = 32 * 32 MHz / bitrate
#[derive(Clone, Copy, defmt::Format)]
pub enum Bitrate {
    /// Arbitrary bitrate in bits per second
    Custom(u32),
}

impl Bitrate {
    /// Get the 3-byte BR register value (FSK formula: 32 * fxosc / bitrate)
    pub fn to_bytes(self) -> [u8; 3] {
        let val = (32u64 * 32_000_000) / self.bps() as u64;
        [(val >> 16) as u8, (val >> 8) as u8, val as u8]
    }

    /// Get the raw bitrate in bps
    pub fn bps(self) -> u32 {
        match self {
            Bitrate::Custom(bps) => bps,
        }
    }
}

/// Gaussian pulse shape filter
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum PulseShape {
    /// No filter applied
    None = 0x00,
    /// Gaussian filter BT 0.3
    GaussianBt03 = 0x08,
    /// Gaussian filter BT 0.5
    GaussianBt05 = 0x09,
    /// Gaussian filter BT 0.7
    GaussianBt07 = 0x0A,
    /// Gaussian filter BT 1.0
    GaussianBt10 = 0x0B,
}

/// FSK receiver bandwidth
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum Bandwidth {
    Bw4_8kHz = 0x1F,
    Bw5_8kHz = 0x17,
    Bw7_3kHz = 0x0F,
    Bw9_7kHz = 0x1E,
    Bw11_7kHz = 0x16,
    Bw14_6kHz = 0x0E,
    Bw19_5kHz = 0x1D,
    Bw23_4kHz = 0x15,
    Bw29_3kHz = 0x0D,
    Bw39kHz = 0x1C,
    Bw46_9kHz = 0x14,
    Bw58_6kHz = 0x0C,
    Bw78_2kHz = 0x1B,
    Bw93_8kHz = 0x13,
    Bw117_3kHz = 0x0B,
    Bw156_2kHz = 0x1A,
    Bw187_2kHz = 0x12,
    Bw234_3kHz = 0x0A,
    Bw312kHz = 0x19,
    Bw373_6kHz = 0x11,
    Bw467kHz = 0x09,
}

/// CRC type for FSK packet params
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum CrcType {
    Off = 0x01,
    /// 1-byte CRC
    Crc1Byte = 0x00,
    /// 2-byte CRC
    Crc2Byte = 0x02,
    /// 1-byte CRC inverted
    Crc1ByteInv = 0x04,
    /// 2-byte CRC inverted
    Crc2ByteInv = 0x06,
}

/// Preamble detection length
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum PreambleDetLength {
    /// Preamble detection disabled
    Off = 0x00,
    /// 8-bit preamble detection
    Bits8 = 0x04,
    /// 16-bit preamble detection
    Bits16 = 0x05,
    /// 24-bit preamble detection
    Bits24 = 0x06,
    /// 32-bit preamble detection
    Bits32 = 0x07,
}

/// Address comparison/filtering mode
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum AddrComp {
    /// Address filtering disabled
    Off = 0x00,
    /// Filter on node address
    Node = 0x01,
    /// Filter on node and broadcast addresses
    NodeBroadcast = 0x02,
}

/// Packet length type
#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum PacketLengthType {
    /// Fixed payload length, no header
    Fixed = 0x00,
    /// Variable payload length, header added to packet
    Variable = 0x01,
}

/// Frequency deviation
/// Formula: register = deviation_hz * 2^25 / 32 MHz
#[derive(Clone, Copy, defmt::Format)]
pub enum FreqDev {
    /// Deviation in Hz
    Hz(u32),
}

impl FreqDev {
    /// Get the 3-byte Fdev register value
    pub fn to_bytes(self) -> [u8; 3] {
        let FreqDev::Hz(hz) = self;
        let val = ((hz as u64) * (1 << 25)) / 32_000_000;
        [(val >> 16) as u8, (val >> 8) as u8, val as u8]
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub struct FskConfig {
    pub frequency: u32,
    pub bitrate: Bitrate,
    pub pulse_shape: PulseShape,
    pub bandwidth: Bandwidth,
    pub fdev: FreqDev,
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

impl Default for FskConfig {
    fn default() -> Self {
        Self {
            frequency: 868_100_000,
            bitrate: Bitrate::Custom(9600),
            pulse_shape: PulseShape::GaussianBt05,
            bandwidth: Bw46_9kHz,
            fdev: FreqDev::Hz(25_000),
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

// Import for default
use Bandwidth::Bw46_9kHz;

/// (G)FSK modulation - borrows a Radio, implements Configure + Transmit + Receive
pub struct FskRadio<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> {
    radio: &'a mut Radio<SPI, TX, RX, EN>,
    payload_len: u8,
    config: FskConfig,
}

impl<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin>
    FskRadio<'a, SPI, TX, RX, EN>
{
    pub fn new(radio: &'a mut Radio<SPI, TX, RX, EN>) -> Self {
        Self {
            radio,
            payload_len: 0,
            config: FskConfig::default(),
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

    /// Send FSK SetPacketParams
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
    for FskRadio<'_, SPI, TX, RX, EN>
{
    type Config = FskConfig;

    async fn configure(&mut self, config: &Self::Config) -> Result<(), RadioError> {
        self.config = *config;

        self.radio.set_packet_type(PacketType::Fsk).await?;

        // Write sync word to SUBGHZ_GSYNCR (0x06C0)
        self.radio.write_register(0x06C0, &config.sync_word).await?;

        // Set FSK packet params
        self.send_packet_params(0).await?;

        // RF frequency
        self.radio.set_rf_frequency(config.frequency).await?;

        // Modulation params
        let br = config.bitrate.to_bytes();
        let fdev = config.fdev.to_bytes();
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
    for FskRadio<'_, SPI, TX, RX, EN>
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
    for FskRadio<'_, SPI, TX, RX, EN>
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
