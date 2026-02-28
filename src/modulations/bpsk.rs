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
pub enum CrcType {
    None,
    /// Using a common 0x07 polynomial
    Crc8,
    /// CRC-16 CCITT using 0x1021 polynomial
    Crc16,
}

impl CrcType {
    fn compute(self, data: &[u8]) -> (u16, usize) {
        match self {
            CrcType::None => (0, 0),
            CrcType::Crc8 => {
                let mut crc: u8 = 0x00;
                for &byte in data {
                    crc ^= byte;
                    for _ in 0..8 {
                        if crc & 0x80 != 0 {
                            crc = (crc << 1) ^ 0x07;
                        } else {
                            crc <<= 1;
                        }
                    }
                }
                (crc as u16, 1)
            }
            CrcType::Crc16 => {
                let mut crc: u16 = 0xFFFF;
                for &byte in data {
                    crc ^= (byte as u16) << 8;
                    for _ in 0..8 {
                        if crc & 0x8000 != 0 {
                            crc = (crc << 1) ^ 0x1021;
                        } else {
                            crc <<= 1;
                        }
                    }
                }
                (crc, 2)
            }
        }
    }

    fn write(self, crc: u16, buf: &mut [u8]) {
        match self {
            CrcType::None => {}
            CrcType::Crc8 => {
                buf[0] = crc as u8;
            }
            CrcType::Crc16 => {
                buf[0] = (crc >> 8) as u8;
                buf[1] = crc as u8;
            }
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Whitening {
    None,
    Ccitt,
}

impl Whitening {
    fn apply(self, seed: u16, data: &mut [u8]) {
        match self {
            Whitening::None => return,
            Whitening::Ccitt => {}
        }

        // Calculate CCITT whitening using x^9 + x^4 + 1 polynomial and LFSR
        let mut lfsr: u16 = seed & 0x1FF;
        for byte in data.iter_mut() {
            let mut mask = 0u8;
            for bit in 0..8 {
                let feedback = ((lfsr >> 8) ^ (lfsr >> 3)) & 1;
                lfsr = ((lfsr << 1) | feedback) & 0x1FF;
                mask |= (feedback as u8) << (7 - bit);
            }
            *byte ^= mask;
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum BpskPacket {
    /// No framing, just send raw data
    Raw,
    /// Use a configurable framing
    Framing {
        /// Length of the preamble (0xAA) in bytes
        preamble_len: usize,
        /// Synchronization word (max 32 bytes)
        sync_word: [u8; 32],
        /// Sync word length
        sync_word_len: usize,
        /// Enable/disable reporting length in the packet
        include_len: bool,
        /// CRC size (0, 1 or 2 bytes)
        crc_type: CrcType,
        /// Whitening algorithm
        whitening: Whitening,
        /// Whitening LFSR seed (9-bit, 0x000..0x1FF)
        whitening_seed: u16,
    },
}

impl BpskPacket {
    pub fn default() -> Self {
        Self::Framing {
            preamble_len: 32,
            // Baker-13 code (2 bytes)
            sync_word: [0x1F, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            sync_word_len: 2,
            include_len: true,
            crc_type: CrcType::Crc16,
            whitening: Whitening::Ccitt,
            whitening_seed: 0x1FF,
        }
    }

    fn to_bytes(self, payload: &[u8], buf: &mut [u8]) -> Result<u8, RadioError> {
        match self {
            BpskPacket::Raw => {
                // Simple copy operation, no modifications made
                buf[..payload.len()].copy_from_slice(payload);
                payload.len().try_into().map_err(|_| RadioError::PayloadTooLarge)
            }
            BpskPacket::Framing {
                preamble_len,
                sync_word,
                sync_word_len,
                include_len,
                crc_type,
                whitening,
                whitening_seed,
            } => {
                let len_field_size = if include_len { 1 } else { 0 };
                let crc_size = match crc_type {
                    CrcType::None => 0,
                    CrcType::Crc8 => 1,
                    CrcType::Crc16 => 2,
                };
                // Validate packet length
                let total = preamble_len + sync_word_len + len_field_size + payload.len() + crc_size;

                if total > buf.len() {
                    return Err(RadioError::PayloadTooLarge);
                }

                // Keeps track of the current position in the buffer
                let mut pos = 0;

                // Write preamble which consists of 0xAA symbols
                buf[pos..pos + preamble_len].fill(0xAA);
                pos += preamble_len;

                // Write sync word
                buf[pos..pos + sync_word_len].copy_from_slice(&sync_word[..sync_word_len]);
                pos += sync_word_len;

                // Actual payload starts here
                let data_start = pos;

                // If enabled in the config, write length info
                if include_len {
                    let payload_len: u8 = payload.len().try_into().map_err(|_| RadioError::PayloadTooLarge)?;
                    buf[pos] = payload_len;
                    pos += 1;
                }

                // Copy the original payload itself
                buf[pos..pos + payload.len()].copy_from_slice(payload);
                pos += payload.len();

                // Compute CRC before the whitening
                let (crc, crc_len) = crc_type.compute(&buf[data_start..pos]);
                crc_type.write(crc, &mut buf[pos..]);
                pos += crc_len;

                // Apply whitening
                whitening.apply(whitening_seed, &mut buf[data_start..pos]);

                // Additional validation - if buffer position can't fit in u8, it's invalid
                pos.try_into().map_err(|_| RadioError::PayloadTooLarge)
            }
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub struct BpskConfig {
    pub frequency: u32,
    pub bitrate: Bitrate,
    pub pa: PaSelection,
    pub power_dbm: i8,
    pub ramp: RampTime,
    pub packet: BpskPacket,
}

impl Default for BpskConfig {
    fn default() -> Self {
        Self {
            frequency: 868_100_000,
            bitrate: Bitrate::Bps600,
            pa: PaSelection::LowPower,
            power_dbm: 14,
            ramp: RampTime::Us40,
            packet: BpskPacket::default(),
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
        let mut buf = [0u8; 255];
        // Convert buffer to packet with chosen framing
        let len = self.config.packet.to_bytes(data, &mut buf)?;

        // Write payload to radio buffer
        self.radio.set_buffer_base(0x00, 0x00).await?;
        self.radio.write_buffer(0x00, &buf[..len as usize]).await?;

        // Update packet params with actual payload length
        self.update_payload_len(len).await?;

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
