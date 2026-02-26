use defmt::debug;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

use crate::{
    RadioError,
    radio::{PaSelection, PacketType, Radio, RampTime, irq},
    traits::{Configure, Transmit},
};

/// BPSK bitrate
/// Formula: register = (32 * 32_000_000) / bps
#[derive(Clone, Copy, defmt::Format)]
pub enum Bitrate {
    /// 100 bits per second
    Bps100,
    /// 600 bits per second
    Bps600,
    /// Arbitrary bitrate in bits per second
    Custom(u32),
}

impl Bitrate {
    /// Get the 3-byte register value for this bitrate
    fn to_bytes(self) -> [u8; 3] {
        let val = match self {
            Bitrate::Bps100 => 0x9C4000,
            Bitrate::Bps600 => 0x1A0AAA,
            Bitrate::Custom(bps) => (32 * 32_000_000) / bps,
        };
        [(val >> 16) as u8, (val >> 8) as u8, val as u8]
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub struct BpskConfig {
    pub frequency: u32,
    pub bitrate: Bitrate,
    pub pa: PaSelection,
    pub power_dbm: i8,
    pub ramp: RampTime,
}

impl Default for BpskConfig {
    fn default() -> Self {
        Self {
            frequency: 868_100_000,
            bitrate: Bitrate::Bps600,
            pa: PaSelection::LowPower,
            power_dbm: 14,
            ramp: RampTime::Us40,
        }
    }
}

/// BPSK modulation - borrows a Radio, implements Configure + Transmit
pub struct BpskRadio<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> {
    radio: &'a mut Radio<SPI, TX, RX, EN>,
    payload_len: u8,
    config: BpskConfig,
}

impl<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin>
    BpskRadio<'a, SPI, TX, RX, EN>
{
    pub fn new(radio: &'a mut Radio<SPI, TX, RX, EN>) -> Self {
        Self {
            radio,
            payload_len: 0,
            config: BpskConfig::default(),
        }
    }

    /// Re-send SetPacketParams with updated payload length (called before each tx/rx)
    async fn update_payload_len(&mut self, len: u8) -> Result<(), RadioError> {
        debug!("Updating payload length to {}", len);
        if len == self.payload_len {
            return Ok(());
        }
        self.payload_len = len;
        self.send_packet_params(len).await
    }

    /// Send the full SetPacketParams command with the given payload length
    async fn send_packet_params(&mut self, payload_len: u8) -> Result<(), RadioError> {
        self.radio.set_packet_params(&[payload_len]).await
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Configure
    for BpskRadio<'_, SPI, TX, RX, EN>
{
    type Config = BpskConfig;

    async fn configure(&mut self, config: &Self::Config) -> Result<(), RadioError> {
        self.config = *config;

        // Select BPSK packet type
        self.radio.set_packet_type(PacketType::Bpsk).await?;

        // Payload length updated per tx
        self.send_packet_params(0).await?;

        // RF frequency
        self.radio.set_rf_frequency(config.frequency).await?;

        // Set modulation params (bitrate + Gaussian BT 0.5 pulse shape)
        let br = config.bitrate.to_bytes();
        self.radio
            .set_modulation_params(&[br[0], br[1], br[2], 0x16])
            .await?;

        // PA config + TX power
        self.radio
            .set_output_power(config.pa, config.power_dbm, config.ramp)
            .await?;

        Ok(())
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Transmit
    for BpskRadio<'_, SPI, TX, RX, EN>
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

        // Enable TxDone IRQ on DIO1
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
