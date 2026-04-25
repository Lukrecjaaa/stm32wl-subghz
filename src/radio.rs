use embassy_stm32::pac;
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{Operation, SpiDevice};

use crate::RadioError;

/// Every opcode that's in RM0461
mod opcode {
    pub const CALIBRATE: u8 = 0x89;
    pub const CALIBRATE_IMAGE: u8 = 0x98;
    pub const CFG_DIO_IRQ: u8 = 0x08;
    pub const CLR_ERROR: u8 = 0x07;
    pub const CLR_IRQ_STATUS: u8 = 0x02;
    pub const GET_ERROR: u8 = 0x17;
    pub const GET_IRQ_STATUS: u8 = 0x12;
    pub const GET_PACKET_STATUS: u8 = 0x14;
    pub const GET_PACKET_TYPE: u8 = 0x11;
    pub const GET_RSSI_INST: u8 = 0x15;
    pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
    pub const GET_STATS: u8 = 0x10;
    pub const GET_STATUS: u8 = 0xC0;
    pub const READ_BUFFER: u8 = 0x1E;
    pub const READ_REGISTER: u8 = 0x1D;
    pub const RESET_STATS: u8 = 0x00;
    pub const SET_BUFFER_BASE_ADDR: u8 = 0x8F;
    pub const SET_CAD: u8 = 0xC5;
    pub const SET_CAD_PARAMS: u8 = 0x88;
    pub const SET_FS: u8 = 0xC1;
    pub const SET_LORA_SYMB_TIMEOUT: u8 = 0xA0;
    pub const SET_MODULATION_PARAMS: u8 = 0x8B;
    pub const SET_PA_CONFIG: u8 = 0x95;
    pub const SET_PACKET_PARAMS: u8 = 0x8C;
    pub const SET_PACKET_TYPE: u8 = 0x8A;
    pub const SET_REGULATOR_MODE: u8 = 0x96;
    pub const SET_RF_FREQUENCY: u8 = 0x86;
    pub const SET_RX: u8 = 0x82;
    pub const SET_RX_DUTY_CYCLE: u8 = 0x94;
    pub const SET_TX_RX_FALLBACK_MODE: u8 = 0x93;
    pub const SET_SLEEP: u8 = 0x84;
    pub const SET_STANDBY: u8 = 0x80;
    pub const SET_STOP_RX_TIMER_ON_PREAMBLE: u8 = 0x9F;
    pub const SET_TCXO_MODE: u8 = 0x97;
    pub const SET_TX: u8 = 0x83;
    pub const SET_TX_CONTINUOUS_WAVE: u8 = 0xD1;
    pub const SET_TX_CONTINUOUS_PREAMBLE: u8 = 0xD2;
    pub const SET_TX_PARAMS: u8 = 0x8E;
    pub const WRITE_BUFFER: u8 = 0x0E;
    pub const WRITE_REGISTER: u8 = 0x0D;
}

/// IRQ bit flags
pub mod irq {
    pub const TX_DONE: u16 = 1 << 0;
    pub const RX_DONE: u16 = 1 << 1;
    pub const PREAMBLE_DETECTED: u16 = 1 << 2;
    pub const SYNC_WORD_VALID: u16 = 1 << 3; // FSK only
    pub const HEADER_VALID: u16 = 1 << 4; // LoRa only
    pub const HEADER_ERR: u16 = 1 << 5; // LoRa only
    pub const CRC_ERR: u16 = 1 << 6;
    pub const CAD_DONE: u16 = 1 << 7; // LoRa only
    pub const CAD_DETECTED: u16 = 1 << 8; // LoRa only
    pub const TIMEOUT: u16 = 1 << 9;
    pub const ALL: u16 = 0x03FF;
}

/// PA ramp time for SetTxParams
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RampTime {
    Us10 = 0x00,
    Us20 = 0x01,
    Us40 = 0x02,
    Us80 = 0x03,
    Us200 = 0x04,
    Us800 = 0x05,
    Us1700 = 0x06,
    Us3400 = 0x07,
}

/// Which power amplifier to use
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PaSelection {
    /// -17 to +14 dBm
    LowPower,
    /// -9 to +22 dBm
    HighPower,
}

/// RX gain setting (register 0x08AC)
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RxGain {
    /// Power saving gain
    PowerSaving = 0x94,
    /// Boosted gain (better sensitivity)
    Boosted = 0x96,
}

/// Standby mode clock source
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum StandbyMode {
    /// Internal 13 MHz RC oscillator
    Rc = 0x00,
    /// External 32 MHz XOSC/TCXO
    Xosc = 0x01,
}

/// Packet type selector
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PacketType {
    Fsk = 0x00,
    LoRa = 0x01,
    Bpsk = 0x02,
    Msk = 0x03,
}

/// Sleep configuration
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SleepConfig {
    /// Start directly in warm start (retain config)
    pub warm_start: bool,
    /// Keep RTC running during sleep
    pub rtc_wakeup: bool,
}

/// Regulator mode
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RegulatorMode {
    /// Only LDO (default after reset)
    Ldo = 0x00,
    /// SMPS mode (more efficient)
    Smps = 0x01,
}

/// Fallback mode after TX/RX completes
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FallbackMode {
    /// Radio goes into STDBY_RC after TX or RX
    StandbyRc = 0x20,
    /// STDBY_XOSC after TX or RX
    StandbyXosc = 0x30,
    /// FS after TX or RX
    Fs = 0x40,
}

/// CAD exit mode
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CadExitMode {
    /// CAD only, return to standby
    CadOnly = 0x00,
    /// CAD then RX if activity detected
    CadRx = 0x01,
}

/// Image calibration frequency ranges
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ImageCalFreq {
    pub freq1: u8,
    pub freq2: u8,
}

impl ImageCalFreq {
    /// 430-440 MHz
    pub const BAND_430: Self = Self {
        freq1: 0x6B,
        freq2: 0x6F,
    };
    /// 470-510 MHz
    pub const BAND_470: Self = Self {
        freq1: 0x75,
        freq2: 0x81,
    };
    /// 779-787 MHz
    pub const BAND_779: Self = Self {
        freq1: 0xC1,
        freq2: 0xC5,
    };
    /// 863-870 MHz
    pub const BAND_863: Self = Self {
        freq1: 0xD7,
        freq2: 0xDB,
    };
    /// 902-928 MHz
    pub const BAND_902: Self = Self {
        freq1: 0xE1,
        freq2: 0xE9,
    };
}

/// LoRa packet status returned by get_packet_status in LoRa mode
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LoraPacketStatus {
    /// RSSI of the last packet in dBm (value / -2)
    pub rssi: i16,
    /// SNR of the last packet in dB (value / 4)
    pub snr: i8,
    /// Signal RSSI in dBm (value / -2)
    pub signal_rssi: i16,
}

/// FSK packet status returned by get_packet_status in FSK mode
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FskPacketStatus {
    pub rx_status: u8,
    /// RSSI when sync word was detected, in dBm (value / -2)
    pub rssi_sync: i16,
    /// Averaged RSSI in dBm (value / -2)
    pub rssi_avg: i16,
}

/// Statistics counters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stats {
    pub packets_received: u16,
    pub packets_crc_error: u16,
    /// Header errors (LoRa) or length errors (FSK)
    pub packets_header_error: u16,
}

/// Tracks which state the radio is in
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RadioState {
    /// Just powered on or reset, before standby
    Startup,
    /// Low power sleep mode
    Sleep,
    /// Calibration in progress
    Calibrating,
    /// Standby mode (RC or XOSC)
    Standby,
    /// Frequency synthesis mode
    Fs,
    /// Transmitting
    Tx,
    /// Receiving
    Rx,
}

pub struct Radio<SPI, TX, RX, EN>
where
    SPI: SpiDevice,
    TX: OutputPin,
    RX: OutputPin,
    EN: OutputPin,
{
    spi: SPI,
    state: RadioState,
    pa: PaSelection,
    rf_tx: TX,
    rf_rx: RX,
    rf_en: EN,
}

impl<SPI, TX, RX, EN> Radio<SPI, TX, RX, EN>
where
    SPI: SpiDevice,
    TX: OutputPin,
    RX: OutputPin,
    EN: OutputPin,
{
    pub fn new(spi: SPI, rf_tx: TX, rf_rx: RX, rf_en: EN) -> Self {
        Self {
            spi,
            state: RadioState::Startup,
            pa: PaSelection::HighPower,
            rf_tx,
            rf_rx,
            rf_en,
        }
    }

    /// Get current radio state
    pub fn state(&self) -> RadioState {
        self.state
    }

    /// Give back the SPI device and RF switch pins
    pub fn release(self) -> (SPI, TX, RX, EN) {
        (self.spi, self.rf_tx, self.rf_rx, self.rf_en)
    }

    // RF switch control
    fn rf_switch_tx(&mut self) {
        let _ = self.rf_tx.set_high();
        let _ = self.rf_rx.set_low();
        let _ = self.rf_en.set_high();
        debug!("RF switch in tx mode");
    }

    fn rf_switch_rx(&mut self) {
        let _ = self.rf_tx.set_low();
        let _ = self.rf_rx.set_high();
        let _ = self.rf_en.set_high();
        debug!("RF switch in rx mode");
    }

    fn rf_switch_off(&mut self) {
        let _ = self.rf_tx.set_low();
        let _ = self.rf_rx.set_low();
        let _ = self.rf_en.set_low();
        debug!("RF switch off");
    }

    /// Ensure the program is in allowed state or return InvalidState
    fn require_state(&self, allowed: &[RadioState]) -> Result<(), RadioError> {
        if allowed.contains(&self.state) {
            Ok(())
        } else {
            Err(RadioError::InvalidState)
        }
    }

    /// Reset the radio via RCC. Transitions to Startup state.
    pub async fn reset(&mut self) -> Result<(), RadioError> {
        debug!("Resetting the radio");
        pac::RCC.csr().modify(|w| w.set_rfrst(true));
        pac::RCC.csr().modify(|w| w.set_rfrst(false));
        Timer::after_millis(1).await;
        self.state = RadioState::Startup;
        debug!("Radio reset finished");
        Ok(())
    }

    /// Enter Standby mode. Allowed from: Startup, Sleep, Fs, Tx, Rx
    pub async fn set_standby(&mut self, mode: StandbyMode) -> Result<(), RadioError> {
        self.require_state(&[
            RadioState::Startup,
            RadioState::Sleep,
            RadioState::Standby,
            RadioState::Fs,
            RadioState::Tx,
            RadioState::Rx,
            RadioState::Calibrating,
        ])?;
        self.cmd(&[opcode::SET_STANDBY, mode as u8]).await?;
        self.rf_switch_off();
        self.state = RadioState::Standby;
        debug!("Radio in Standby mode");
        Ok(())
    }

    /// Enter Sleep mode. Allowed from: Standby
    pub async fn set_sleep(&mut self, config: SleepConfig) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        let byte = (config.warm_start as u8) << 2 | (config.rtc_wakeup as u8);
        self.cmd(&[opcode::SET_SLEEP, byte]).await?;
        self.rf_switch_off();
        self.state = RadioState::Sleep;
        debug!("Radio in Sleep mode");
        Ok(())
    }

    /// Enter Fs mode. Allowed from: Standby
    pub async fn set_fs(&mut self) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_FS]).await?;
        self.state = RadioState::Fs;
        debug!("Radio in Fs mode");
        Ok(())
    }

    /// Start tx, timeout in 15.625µs steps, 0 = no timeout
    /// Allowed from: Standby (goes through FS automatically)
    pub async fn set_tx(&mut self, timeout: u32) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby, RadioState::Fs])?;
        self.rf_switch_tx();
        self.cmd(&[
            opcode::SET_TX,
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8,
        ])
        .await?;
        self.state = RadioState::Tx;
        debug!("Radio in Tx mode");
        Ok(())
    }

    /// Start rx, timeout in 15.625µs steps, 0 = single, 0xFFFFFF = continuous
    /// Allowed from: Standby (goes through FS automatically)
    pub async fn set_rx(&mut self, timeout: u32) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby, RadioState::Fs])?;
        self.rf_switch_rx();
        self.cmd(&[
            opcode::SET_RX,
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8,
        ])
        .await?;
        self.state = RadioState::Rx;
        debug!("Radio in Rx mode");
        Ok(())
    }

    /// Alternates between RX and sleep
    /// Allowed from: Standby
    pub async fn set_rx_duty_cycle(
        &mut self,
        rx_period: u32,
        sleep_period: u32,
    ) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.rf_switch_rx();
        self.cmd(&[
            opcode::SET_RX_DUTY_CYCLE,
            (rx_period >> 16) as u8,
            (rx_period >> 8) as u8,
            rx_period as u8,
            (sleep_period >> 16) as u8,
            (sleep_period >> 8) as u8,
            sleep_period as u8,
        ])
        .await?;
        self.state = RadioState::Rx;
        debug!("Radio in Rx mode");
        Ok(())
    }

    /// Start channel activity detection. Allowed from: Standby
    pub async fn set_cad(&mut self) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.rf_switch_rx();
        self.cmd(&[opcode::SET_CAD]).await?;
        self.state = RadioState::Rx; // CAD uses the receiver
        debug!("Radio in Rx with CAD enabled");
        Ok(())
    }

    /// Start continuous wave output (for testing). Allowed from: Standby
    pub async fn set_tx_continuous_wave(&mut self) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.rf_switch_tx();
        self.cmd(&[opcode::SET_TX_CONTINUOUS_WAVE]).await?;
        self.state = RadioState::Tx;
        debug!("Radio in Tx mode (continuous wave");
        Ok(())
    }

    /// Start continuous preamble (for testing). Allowed from: Standby.
    pub async fn set_tx_continuous_preamble(&mut self) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.rf_switch_tx();
        self.cmd(&[opcode::SET_TX_CONTINUOUS_PREAMBLE]).await?;
        self.state = RadioState::Tx;
        debug!("Radio in Tx mode (continuous preamble");
        Ok(())
    }

    /// Configure DIO3 to power the TCXO
    /// voltage: 0x01 = 1.7V, timeout in 15.625µs steps
    /// Allowed from: Standby
    pub async fn set_tcxo(&mut self, voltage: u8, timeout: u32) -> Result<(), RadioError> {
        debug!(
            "Trying DIO3 powering up TCX0, voltage {}, timeout {}",
            voltage, timeout
        );
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[
            opcode::SET_TCXO_MODE,
            voltage,
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8,
        ])
        .await
    }

    /// Calibrate internal blocks. mask bits:
    /// bit 0: RC64k, bit 1: RC13M, bit 2: PLL, bit 3: ADC pulse,
    /// bit 4: ADC bulk N, bit 5: ADC bulk P, bit 6: image
    /// 0x7F = all
    /// Allowed from: Standby
    pub async fn calibrate(&mut self, mask: u8) -> Result<(), RadioError> {
        self.require_state(&[RadioState::Standby])?;
        self.state = RadioState::Calibrating;
        self.cmd(&[opcode::CALIBRATE, mask]).await?;
        // Radio returns to standby after calibration
        self.state = RadioState::Standby;
        debug!("Radio in Standby mode, internal blocks calibrated");
        Ok(())
    }

    /// Calibrate image rejection for a frequency band
    /// Allowed from: Standby
    pub async fn calibrate_image(&mut self, band: ImageCalFreq) -> Result<(), RadioError> {
        debug!("Calibrating image rejection for band {}", band);
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::CALIBRATE_IMAGE, band.freq1, band.freq2])
            .await
    }

    /// Full startup: reset, standby, SMPS, TCXO, calibrate, retention list
    pub async fn init(&mut self) -> Result<(), RadioError> {
        debug!("Starting radio init");
        self.reset().await?;
        // Wake radio with GetStatus
        self.cmd(&[opcode::GET_STATUS, 0x00]).await?;
        self.set_standby(StandbyMode::Rc).await?;
        // SMPS regulator mode (more efficient than LDO)
        self.set_regulator_mode(RegulatorMode::Smps).await?;
        // Clear XOSC_START_ERR (expected after POR with TCXO)
        self.clear_error().await?;
        // TCXO 1.7V, 10ms timeout (0x000280 = 640 * 15.625µs)
        self.set_tcxo(0x01, 0x000280).await?;
        // Calibrate all blocks
        self.calibrate(0x7F).await?;
        Timer::after_millis(10).await;
        // Set buffer base addresses
        self.set_buffer_base(0x00, 0x00).await?;
        // Update retention list so RxGain and TxModulation survive sleep
        self.update_retention_list().await?;
        // Default PA + TX params (needed even for RX to fully initialize the radio)
        self.set_pa_config(0x02, 0x02, 0x00).await?;
        self.set_tx_params(0, RampTime::Us200).await?;
        // Set all IRQs on DIO1 (will be reconfigured per tx/rx)
        self.set_dio1_irq(irq::ALL).await?;
        debug!("Radio is initialized");
        Ok(())
    }

    /// Get the appropriate ImageCalFreq for a given frequency in Hz
    pub fn image_cal_for_freq(freq_hz: u32) -> ImageCalFreq {
        if freq_hz < 446_000_000 {
            ImageCalFreq::BAND_430
        } else if freq_hz < 600_000_000 {
            ImageCalFreq::BAND_470
        } else if freq_hz < 790_000_000 {
            ImageCalFreq::BAND_779
        } else if freq_hz < 880_000_000 {
            ImageCalFreq::BAND_863
        } else {
            ImageCalFreq::BAND_902
        }
    }

    /// Add a register to the retention list (survives sleep mode warm start)
    async fn add_to_retention_list(&mut self, addr_hi: u8, addr_lo: u8) -> Result<(), RadioError> {
        debug!(
            "Adding register {:x}{:x} to retention list",
            addr_hi, addr_lo
        );
        let mut ret = [0u8; 9];
        self.read_register(0x029F, &mut ret).await?;
        let count = ret[0] as usize;
        if count >= 4 {
            return Ok(()); // retention list full (max 4 entries)
        }
        ret[0] += 1;
        ret[1 + count * 2] = addr_hi;
        ret[2 + count * 2] = addr_lo;
        self.write_register(0x029F, &ret).await
    }

    /// Update retention list with RxGain (0x08AC) and TxModulation (0x0889)
    async fn update_retention_list(&mut self) -> Result<(), RadioError> {
        debug!("Updating retention list");
        self.add_to_retention_list(0x08, 0xAC).await?;
        self.add_to_retention_list(0x08, 0x89).await
    }

    /// Set the packet type (LoRa, FSK, BPSK, MSK)
    pub async fn set_packet_type(&mut self, pkt_type: PacketType) -> Result<(), RadioError> {
        debug!("Setting packet type {}", pkt_type);
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_PACKET_TYPE, pkt_type as u8]).await
    }

    /// Get the current packet type
    pub async fn get_packet_type(&mut self) -> Result<PacketType, RadioError> {
        let mut buf = [0u8; 2];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_PACKET_TYPE]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        match buf[1] {
            0x00 => Ok(PacketType::Fsk),
            0x01 => Ok(PacketType::LoRa),
            0x02 => Ok(PacketType::Bpsk),
            0x03 => Ok(PacketType::Msk),
            _ => Err(RadioError::Spi),
        }
    }

    /// Set RF frequency in Hz
    /// Calculates PLL steps: rffreq = (freq_hz * 2^25) / 32_000_000
    pub async fn set_rf_frequency(&mut self, freq_hz: u32) -> Result<(), RadioError> {
        debug!("Setting frequency to {}", freq_hz);
        self.require_state(&[RadioState::Standby, RadioState::Fs])?;
        let frf = ((freq_hz as u64) << 25) / 32_000_000;
        self.cmd(&[
            opcode::SET_RF_FREQUENCY,
            (frf >> 24) as u8,
            (frf >> 16) as u8,
            (frf >> 8) as u8,
            frf as u8,
        ])
        .await
    }

    /// Configure the power amplifier (raw)
    pub async fn set_pa_config(
        &mut self,
        duty_cycle: u8,
        hp_max: u8,
        pa_sel: u8,
    ) -> Result<(), RadioError> {
        debug!(
            "Setting Pa config to duty cycle {:x} hp_max {:x} pa_sel {:x}",
            duty_cycle, hp_max, pa_sel
        );
        self.require_state(&[RadioState::Standby])?;
        self.pa = if pa_sel == 0x01 {
            PaSelection::LowPower
        } else {
            PaSelection::HighPower
        };
        self.cmd(&[opcode::SET_PA_CONFIG, duty_cycle, hp_max, pa_sel, 0x01])
            .await
    }

    /// Set TX output power and PA ramp time (raw)
    pub async fn set_tx_params(&mut self, power_dbm: i8, ramp: RampTime) -> Result<(), RadioError> {
        debug!("Setting tx params power_dbm {} ramp {}", power_dbm, ramp);
        self.require_state(&[RadioState::Standby])?;
        match self.pa {
            PaSelection::LowPower if power_dbm < -17 || power_dbm > 15 => {
                return Err(RadioError::InvalidConfig);
            }
            PaSelection::HighPower if power_dbm < -9 || power_dbm > 22 => {
                return Err(RadioError::InvalidConfig);
            }
            _ => {}
        }
        self.cmd(&[opcode::SET_TX_PARAMS, power_dbm as u8, ramp as u8])
            .await
    }

    /// Set output power using the optimal PA configuration from the datasheet
    /// Picks the right PA mode, duty cycle, hp_max, and power register value
    /// Supported LP levels: +10, +14, +15 dBm
    /// Supported HP levels: +14, +17, +20, +22 dBm
    pub async fn set_output_power(
        &mut self,
        pa: PaSelection,
        power_dbm: i8,
        ramp: RampTime,
    ) -> Result<(), RadioError> {
        let (duty_cycle, hp_max, pa_sel, power) = match (pa, power_dbm) {
            // LP PA (PaSel = 1)
            (PaSelection::LowPower, 15) => (0x07, 0x00, 0x01, 0x0E_i8),
            (PaSelection::LowPower, 14) => (0x04, 0x00, 0x01, 0x0E),
            (PaSelection::LowPower, 10) => (0x01, 0x00, 0x01, 0x0D),
            // HP PA (PaSel = 0)
            (PaSelection::HighPower, 22) => (0x04, 0x07, 0x00, 0x16),
            (PaSelection::HighPower, 20) => (0x03, 0x05, 0x00, 0x16),
            (PaSelection::HighPower, 17) => (0x02, 0x03, 0x00, 0x16),
            (PaSelection::HighPower, 14) => (0x02, 0x02, 0x00, 0x16),
            _ => return Err(RadioError::InvalidConfig),
        };
        self.set_pa_config(duty_cycle, hp_max, pa_sel).await?;
        self.set_tx_params(power, ramp).await
    }

    /// Set modulation params (raw bytes, meaning depends on packet type)
    /// LoRa: [SF, BW, CR, LDRO]
    /// FSK: [BR2, BR1, BR0, PulseShape, BW, FDEV2, FDEV1, FDEV0]
    /// BPSK: [BR2, BR1, BR0, PulseShape]
    pub async fn set_modulation_params(&mut self, params: &[u8]) -> Result<(), RadioError> {
        debug!("Setting modulation params {:x}", params);
        self.require_state(&[RadioState::Standby])?;
        let mut buf = [0u8; 9];
        buf[0] = opcode::SET_MODULATION_PARAMS;
        let len = params.len().min(8);
        buf[1..1 + len].copy_from_slice(&params[..len]);
        self.cmd(&buf[..1 + len]).await
    }

    /// Set packet params (raw bytes, meaning depends on packet type)
    /// LoRa: [PreambleH, PreambleL, HeaderType, PayloadLen, CrcType, InvertIQ]
    /// FSK: [PreambleH, PreambleL, PreambleDetLen, SyncWordLen, AddrComp, PktType, PayloadLen, CrcType, Whitening]
    /// BPSK: [PayloadLen]
    pub async fn set_packet_params(&mut self, params: &[u8]) -> Result<(), RadioError> {
        debug!("Setting packet params {:x}", params);
        self.require_state(&[RadioState::Standby])?;
        let mut buf = [0u8; 10];
        buf[0] = opcode::SET_PACKET_PARAMS;
        let len = params.len().min(9);
        buf[1..1 + len].copy_from_slice(&params[..len]);
        self.cmd(&buf[..1 + len]).await
    }

    /// Set CAD parameters (LoRa only)
    pub async fn set_cad_params(
        &mut self,
        nb_symbol: u8,
        det_peak: u8,
        det_min: u8,
        exit_mode: CadExitMode,
        timeout: u32,
    ) -> Result<(), RadioError> {
        debug!(
            "Setting CAD params nb_symbol {:x} det_peak {:x} det_min {:x} exit_mode {:x} timeout {}",
            nb_symbol, det_peak, det_min, exit_mode, timeout
        );
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[
            opcode::SET_CAD_PARAMS,
            nb_symbol,
            det_peak,
            det_min,
            exit_mode as u8,
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8,
        ])
        .await
    }

    /// Set LoRa symbol timeout for RX
    pub async fn set_lora_symb_timeout(&mut self, symb_num: u8) -> Result<(), RadioError> {
        debug!("Setting LoRa symbol timeout symb_num {:x}", symb_num);
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_LORA_SYMB_TIMEOUT, symb_num]).await
    }

    /// Set regulator mode (LDO only or SMPS)
    pub async fn set_regulator_mode(&mut self, mode: RegulatorMode) -> Result<(), RadioError> {
        debug!("Setting regulator mode to {}", mode);
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_REGULATOR_MODE, mode as u8]).await
    }

    /// Set fallback mode after TX/RX completes
    pub async fn set_tx_rx_fallback_mode(&mut self, mode: FallbackMode) -> Result<(), RadioError> {
        debug!("Setting fallback mode to {}", mode);
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_TX_RX_FALLBACK_MODE, mode as u8])
            .await
    }

    /// Stop RX timer on preamble detection (true) or on sync/header (false)
    pub async fn set_stop_rx_timer_on_preamble(
        &mut self,
        on_preamble: bool,
    ) -> Result<(), RadioError> {
        debug!(
            "Setting rx timer on preamble detect (true) or sync/header (false) to {}",
            on_preamble
        );
        self.require_state(&[RadioState::Standby])?;
        self.cmd(&[opcode::SET_STOP_RX_TIMER_ON_PREAMBLE, on_preamble as u8])
            .await
    }

    /// Set TX and RX buffer base addresses in the 256-byte radio buffer
    pub async fn set_buffer_base(&mut self, tx_base: u8, rx_base: u8) -> Result<(), RadioError> {
        debug!(
            "Setting buffer base to tx_base {:x} rx_base {:x}",
            tx_base, rx_base
        );
        self.cmd(&[opcode::SET_BUFFER_BASE_ADDR, tx_base, rx_base])
            .await
    }

    /// Write payload data to the radio buffer
    pub async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), RadioError> {
        trace!(
            "Writing payload to radio buffer at offset {:x}: {:x}",
            offset, data
        );
        if data.len() > 255 {
            return Err(RadioError::PayloadTooLarge);
        }
        let header = [opcode::WRITE_BUFFER, offset];
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
            .await
            .map_err(|_| RadioError::Spi)
    }

    /// Read data from the radio buffer
    pub async fn read_buffer(&mut self, offset: u8, buf: &mut [u8]) -> Result<(), RadioError> {
        let header = [opcode::READ_BUFFER, offset, 0x00]; // extra NOP for status
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(buf)])
            .await
            .map_err(|_| RadioError::Spi)
    }

    /// Write to a 16-bit addressed register
    pub async fn write_register(&mut self, addr: u16, data: &[u8]) -> Result<(), RadioError> {
        trace!("Writing payload to register at addr {:x}: {:x}", addr, data);
        let header = [opcode::WRITE_REGISTER, (addr >> 8) as u8, addr as u8];
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
            .await
            .map_err(|_| RadioError::Spi)
    }

    /// Read from a 16-bit addressed register
    pub async fn read_register(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), RadioError> {
        let header = [opcode::READ_REGISTER, (addr >> 8) as u8, addr as u8, 0x00]; // NOP for status
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(buf)])
            .await
            .map_err(|_| RadioError::Spi)
    }

    /// Write LoRa sync word to SUBGHZ_LSYNCR (0x0740)
    pub async fn set_lora_sync_word(&mut self, sync_word: u16) -> Result<(), RadioError> {
        debug!("Writing lora sync word to SUBGHZ_LSYNCR: {:x}", sync_word);
        self.write_register(0x0740, &[(sync_word >> 8) as u8, sync_word as u8])
            .await
    }

    /// Configure which IRQs are enabled and routed to DIO1/DIO2/DIO3
    pub async fn set_dio_irq(
        &mut self,
        irq_mask: u16,
        dio1_mask: u16,
        dio2_mask: u16,
        dio3_mask: u16,
    ) -> Result<(), RadioError> {
        debug!(
            "Setting DIO IRQs, masks: irq {:x} dio1 {:x} dio2 {:x} dio3 {:x}",
            irq_mask, dio1_mask, dio2_mask, dio3_mask
        );
        self.cmd(&[
            opcode::CFG_DIO_IRQ,
            (irq_mask >> 8) as u8,
            irq_mask as u8,
            (dio1_mask >> 8) as u8,
            dio1_mask as u8,
            (dio2_mask >> 8) as u8,
            dio2_mask as u8,
            (dio3_mask >> 8) as u8,
            dio3_mask as u8,
        ])
        .await
    }

    /// Enable IRQs on DIO1 only (most common case on STM32WL)
    pub async fn set_dio1_irq(&mut self, mask: u16) -> Result<(), RadioError> {
        debug!("Setting IRQs on DIO1, mask {:x}", mask);
        self.set_dio_irq(mask, mask, 0, 0).await
    }

    /// Read current IRQ status flags
    pub async fn get_irq_status(&mut self) -> Result<u16, RadioError> {
        let mut buf = [0u8; 3];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_IRQ_STATUS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        Ok(u16::from_be_bytes([buf[1], buf[2]]))
    }

    /// Clear IRQ flags
    pub async fn clear_irq(&mut self, mask: u16) -> Result<(), RadioError> {
        debug!("Clearing IRQs, mask {:x}", mask);
        self.cmd(&[opcode::CLR_IRQ_STATUS, (mask >> 8) as u8, mask as u8])
            .await
    }

    /// Poll until any bit in mask is set, clear all IRQs, return what fired
    /// Additionally, radio auto-returns after TX/RX to Standby
    pub async fn poll_irq(&mut self, mask: u16) -> Result<u16, RadioError> {
        debug!("Polling IRQ, mask {:x}", mask);
        loop {
            let status = self.get_irq_status().await?;
            trace!("Got IRQ status {:x}", status);
            if status & mask != 0 {
                self.clear_irq(irq::ALL).await?;
                // Radio returns to standby after TX/RX completes
                if status & (irq::TX_DONE | irq::RX_DONE | irq::TIMEOUT) != 0 {
                    self.rf_switch_off();
                    self.state = RadioState::Standby;
                    debug!("Radio in Standby mode");
                }
                return Ok(status);
            }
            Timer::after_millis(1).await;
        }
    }

    /// Get radio status byte
    pub async fn get_status(&mut self) -> Result<u8, RadioError> {
        let mut buf = [0u8; 1];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_STATUS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        debug!("Got status {:x}", buf[0]);
        Ok(buf[0])
    }

    /// Get RX buffer status: (payload_length, buffer_offset)
    pub async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError> {
        let mut buf = [0u8; 3];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_RX_BUFFER_STATUS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        debug!("Got RX buffer status {:x} {:x}", buf[1], buf[2]);
        Ok((buf[1], buf[2]))
    }

    /// Get LoRa packet status (RSSI, SNR, signal RSSI)
    pub async fn get_lora_packet_status(&mut self) -> Result<LoraPacketStatus, RadioError> {
        let mut buf = [0u8; 4];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_PACKET_STATUS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        let rssi = -(buf[1] as i16) / 2;
        let snr = (buf[2] as i8) / 4;
        let signal_rssi = -(buf[3] as i16) / 2;
        debug!(
            "Got LoRa packet status rssi {} snr {} signal_rssi {}",
            rssi, snr, signal_rssi
        );
        Ok(LoraPacketStatus {
            rssi,
            snr,
            signal_rssi,
        })
    }

    /// Get FSK packet status
    pub async fn get_fsk_packet_status(&mut self) -> Result<FskPacketStatus, RadioError> {
        let mut buf = [0u8; 4];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_PACKET_STATUS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        let rx_status = buf[1];
        let rssi_sync = -(buf[2] as i16) / 2;
        let rssi_avg = -(buf[3] as i16) / 2;
        debug!(
            "Got FSK packet status rx_status {:x} rssi_sync {} rssi_avg {}",
            rx_status, rssi_sync, rssi_avg
        );
        Ok(FskPacketStatus {
            rx_status,
            rssi_sync,
            rssi_avg,
        })
    }

    /// Get instantaneous RSSI in dBm
    pub async fn get_rssi_inst(&mut self) -> Result<i16, RadioError> {
        let mut buf = [0u8; 2];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_RSSI_INST]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        let rssi_inst = -(buf[1] as i16) / 2;
        debug!("Got inst rssi {} dBm", rssi_inst);
        Ok(rssi_inst)
    }

    /// Get packet statistics counters
    /// Field meanings depend on packet type:
    /// - LoRa: packets_header_error = header CRC errors
    /// - FSK: packets_header_error = payload length errors
    pub async fn get_stats(&mut self) -> Result<Stats, RadioError> {
        let mut buf = [0u8; 7];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_STATS]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        // buf[0] = Status, then 3 big-endian u16 values
        let packets_received = u16::from_be_bytes([buf[1], buf[2]]);
        let packets_crc_error = u16::from_be_bytes([buf[3], buf[4]]);
        let packets_header_error = u16::from_be_bytes([buf[5], buf[6]]);

        debug!(
            "Got packet stats counters: status {:x} packets_recv {} packets_crc_err {} header crc err (lora)/payload len err (fsk) {}",
            buf[0], packets_received, packets_crc_error, packets_header_error
        );
        Ok(Stats {
            packets_received,
            packets_crc_error,
            packets_header_error,
        })
    }

    /// Reset packet statistics counters
    pub async fn reset_stats(&mut self) -> Result<(), RadioError> {
        debug!("Resetting packet stats counters");
        self.cmd(&[opcode::RESET_STATS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            .await
    }

    /// Get error flags from the radio
    pub async fn get_error(&mut self) -> Result<u16, RadioError> {
        let mut buf = [0u8; 3];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_ERROR]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        let err_flag = u16::from_be_bytes([buf[1], buf[2]]);
        debug!("Got error flag {:x}", err_flag);
        Ok(err_flag)
    }

    /// Clear error flags (reads back status as per datasheet)
    pub async fn clear_error(&mut self) -> Result<(), RadioError> {
        debug!("Clearing error flags");
        let mut buf = [0u8; 2];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::CLR_ERROR]),
                Operation::Read(&mut buf),
            ])
            .await
            .map_err(|_| RadioError::Spi)?;
        Ok(())
    }

    async fn cmd(&mut self, data: &[u8]) -> Result<(), RadioError> {
        self.spi.write(data).await.map_err(|_| RadioError::Spi)
    }
}
