use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiDevice;

use crate::error::RadioError;
use crate::radio::{PaSelection, PacketType, Radio, RampTime, RxGain, irq};
use crate::traits::{Configure, Receive, Transmit};

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SpreadingFactor {
    SF5 = 0x05,
    SF6 = 0x06,
    SF7 = 0x07,
    SF8 = 0x08,
    SF9 = 0x09,
    SF10 = 0x0a,
    SF11 = 0x0b,
    SF12 = 0x0c,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Bandwidth {
    Bw7_8kHz = 0x00,
    Bw10_42kHz = 0x08,
    Bw15_63kHz = 0x01,
    Bw20_83kHz = 0x09,
    Bw31_25kHz = 0x02,
    Bw41_67kHz = 0x0a,
    Bw62_50kHz = 0x03,
    Bw125kHz = 0x04,
    Bw250kHz = 0x05,
    Bw500kHz = 0x06,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CodingRate {
    /// No forward error correction coding
    Cr44 = 0x00,
    Cr45 = 0x01,
    Cr46 = 0x02,
    Cr47 = 0x03,
    Cr48 = 0x04,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LoraConfig {
    pub frequency: u32,
    pub sf: SpreadingFactor,
    pub bw: Bandwidth,
    pub cr: CodingRate,
    pub ldro: bool,
    pub preamble_len: u16,
    pub explicit_header: bool,
    pub crc_on: bool,
    pub iq_inverted: bool,
    pub sync_word: u16,
    pub rx_gain: RxGain,
    pub pa: PaSelection,
    pub power_dbm: i8,
    pub ramp: RampTime,
}

impl Default for LoraConfig {
    fn default() -> Self {
        Self {
            frequency: 868_100_000,
            sf: SpreadingFactor::SF7,
            bw: Bandwidth::Bw125kHz,
            cr: CodingRate::Cr45,
            ldro: false,
            preamble_len: 8,
            explicit_header: true,
            crc_on: true,
            iq_inverted: false,
            sync_word: 0x1424, // private LoRa network
            rx_gain: RxGain::PowerSaving,
            pa: PaSelection::LowPower,
            power_dbm: 14,
            ramp: RampTime::Us40,
        }
    }
}

/// LoRa modulation - borrows a Radio, implements Configure + Transmit + Receive
pub struct LoraRadio<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> {
    radio: &'a mut Radio<SPI, TX, RX, EN>,
    payload_len: u8,
    config: LoraConfig,
}

impl<'a, SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin>
    LoraRadio<'a, SPI, TX, RX, EN>
{
    pub fn new(radio: &'a mut Radio<SPI, TX, RX, EN>) -> Self {
        Self {
            radio,
            payload_len: 0,
            config: LoraConfig::default(),
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
        self.radio
            .set_packet_params(&[
                (self.config.preamble_len >> 8) as u8,
                self.config.preamble_len as u8,
                !self.config.explicit_header as u8,
                payload_len,
                self.config.crc_on as u8,
                self.config.iq_inverted as u8,
            ])
            .await
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Configure
    for LoraRadio<'_, SPI, TX, RX, EN>
{
    type Config = LoraConfig;

    async fn configure(&mut self, config: &LoraConfig) -> Result<(), RadioError> {
        self.config = *config;

        // Select LoRa packet type
        self.radio.set_packet_type(PacketType::LoRa).await?;

        // Calibrate image for this frequency band
        let band = Radio::<SPI, TX, RX, EN>::image_cal_for_freq(config.frequency);
        self.radio.calibrate_image(band).await?;

        // RF frequency
        self.radio.set_rf_frequency(config.frequency).await?;

        // Modulation: SF, BW, CR, LDRO
        self.radio
            .set_modulation_params(&[
                config.sf as u8,
                config.bw as u8,
                config.cr as u8,
                config.ldro as u8,
            ])
            .await?;

        // Packet params (payload length 0 for now, updated per tx/rx)
        self.send_packet_params(0).await?;
        self.payload_len = 0;

        // Fix IQ polarity for non-inverted IQ (set bit 2 of register 0x0736)
        if !config.iq_inverted {
            let mut iqpol = [0u8; 1];
            self.radio.read_register(0x0736, &mut iqpol).await?;
            trace!("Got data {:x}", iqpol);
            self.radio
                .write_register(0x0736, &[iqpol[0] | 0x04])
                .await?;
        }

        // Sync word at SUBGHZ_LSYNCR (0x0740)
        self.radio.set_lora_sync_word(config.sync_word).await?;

        // PA config + TX power (uses optimal settings from the datasheet)
        self.radio
            .set_output_power(config.pa, config.power_dbm, config.ramp)
            .await?;

        Ok(())
    }
}

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Transmit
    for LoraRadio<'_, SPI, TX, RX, EN>
{
    async fn tx(&mut self, data: &[u8]) -> Result<(), RadioError> {
        if data.len() > 255 {
            return Err(RadioError::PayloadTooLarge);
        }

        // Write payload to radio buffer
        self.radio.set_buffer_base(0x00, 0x80).await?;
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

impl<SPI: SpiDevice, TX: OutputPin, RX: OutputPin, EN: OutputPin> Receive
    for LoraRadio<'_, SPI, TX, RX, EN>
{
    async fn rx(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, RadioError> {
        // Set max payload length we can accept
        let max_len = buf.len().min(255) as u8;
        self.update_payload_len(max_len).await?;

        // Set buffer base addresses (both at 0, same as in lora-rs)
        self.radio.set_buffer_base(0x00, 0x00).await?;

        // Clear any stale IRQ flags before starting RX
        self.radio.clear_irq(irq::ALL).await?;

        // Enable RX-related IRQs on DIO1
        self.radio
            .set_dio1_irq(irq::RX_DONE | irq::TIMEOUT | irq::CRC_ERR | irq::HEADER_ERR)
            .await?;

        // Stop RX timer on preamble detection (required for proper RX behavior)
        self.radio.set_stop_rx_timer_on_preamble(true).await?;

        // Set RX gain
        self.radio
            .write_register(0x08AC, &[self.config.rx_gain as u8])
            .await?;

        // Convert ms to 15.625µs steps (ms * 64), 0 = single mode, 0xFFFFFF = continuous
        let timeout_steps = if timeout_ms == 0 {
            0
        } else {
            timeout_ms.saturating_mul(64).min(0xFFFFFF)
        };
        self.radio.set_rx(timeout_steps).await?;

        // Wait for something to happen
        let status = self
            .radio
            .poll_irq(irq::RX_DONE | irq::TIMEOUT | irq::CRC_ERR | irq::HEADER_ERR)
            .await?;

        // Check what happened
        if status & irq::TIMEOUT != 0 {
            return Err(RadioError::Timeout);
        }
        if status & irq::CRC_ERR != 0 {
            return Err(RadioError::CrcInvalid);
        }
        if status & irq::HEADER_ERR != 0 {
            return Err(RadioError::HeaderInvalid);
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
