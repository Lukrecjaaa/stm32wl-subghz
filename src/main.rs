#![no_std]
#![no_main]

use defmt::{debug, trace};
use embassy_executor::Spawner;
use embassy_stm32::{Config, gpio::{Level, Output, Speed}, pac, rcc::{MSIRange, Sysclk, mux}, spi::Spi};
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use {defmt_rtt as _, panic_probe as _};

/// Wrapper for the sub-GHz SPI device
struct SubGhzSpiDevice<T>(T);

impl<T: SpiBus> ErrorType for SubGhzSpiDevice<T> {
    type Error = T::Error;
}

/// This works as a translation layer between normal SPI transactions and sub-GHz device SPI
/// transactions. Everything above this layer sees it like a normal SPI device!
impl<T: SpiBus> SpiDevice for SubGhzSpiDevice<T> {
    /// Perform a transaction on the sub-GHz device
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        // Pull NSS low to allow SPI comms
        pac::PWR.subghzspicr().modify(|w| w.set_nss(false));
        trace!("NSS low");
        for operation in operations {
            match operation {
                Operation::Read(buf) => {
                    self.0.read(buf).await?;
                    trace!("Read {:x}", buf);
                },
                Operation::Write(buf) => {
                    self.0.write(buf).await?;
                    trace!("Wrote {:x}", buf);
                },
                Operation::Transfer(read, write) => {
                    self.0.transfer(read, write).await?;
                    trace!("Read {:x} wrote {:x}", read, write);
                },
                Operation::TransferInPlace(buf) => {
                    self.0.transfer_in_place(buf).await?;
                    trace!("Read+wrote {:x}", buf);
                },
                Operation::DelayNs(_) => {}
            }
        }
        // Pull NSS high
        pac::PWR.subghzspicr().modify(|w| w.set_nss(true));
        trace!("NSS high");
        // Poll BUSY flag until it's done
        while pac::PWR.sr2().read().rfbusys() {}
        trace!("BUSY flag clear");
        Ok(())
    }
}

async fn reset_radio() {
    debug!("Resetting the radio");
    pac::RCC.csr().modify(|w| w.set_rfrst(true));
    pac::RCC.csr().modify(|w| w.set_rfrst(false));
    Timer::after_millis(1).await;
    debug!("Radio reset finished");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        config.rcc.msi = Some(MSIRange::RANGE48M);
        config.rcc.sys = Sysclk::MSI;
        config.rcc.mux.rngsel = mux::Rngsel::MSI;
        config.enable_debug_during_sleep = true;
    }
    let p = embassy_stm32::init(config);

    let mut spi = SubGhzSpiDevice(Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2));
    reset_radio().await;

    debug!("Writing SetStandby");
    let _ = spi.write(&[0x80, 0x00]).await;
    debug!("Radio in standby!");

    // SetDIO3AsTCXOCtrl - 1.7V, 5ms timeout
    debug!("SetDIO3AsTCXOCtrl");
    let _ = spi.write(&[0x97, 0x01, 0x00, 0x01, 0x45]).await;
    // Calibrate - all blocks (RC64k, RC13M, PLL, ADC pulse, ADC bulk N, ADC bulk P, image)
    debug!("Calibrate");
    let _ = spi.write(&[0x89, 0x7F]).await;
    // CalibrateImage - for 863-870 MHz band
    debug!("CalibrateImage");
    let _ = spi.write(&[0x98, 0xD7, 0xDB]).await;
    // SetBufferBaseAddress
    debug!("SetBufferBaseAddress");
    let _ = spi.write(&[0x8f, 0x00, 0x00]).await;
    // WriteBuffer (max 255 bytes, wraps around after that)
    debug!("WriteBuffer");
    let _ = spi.write(&[0x0e, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05]).await;
    // SetPacketType to LoRa
    debug!("LoRa SetPacketType");
    let _ = spi.write(&[0x8a, 0x01]).await;
    // SetPacketParam - 8 preamble length, explicit header, 5-byte payload,
    // CRC enabled, standard IQ setup
    debug!("SetPacketParam");
    let _ = spi.write(&[0x8c, 0x00, 0x08, 0x00, 0x05, 0x01, 0x00]).await;
    // Probably redundant, but:
    // Defining LoRa sync word (public network) with WriteRegister to SUBGHZ_LSYNCR (0x740)
    debug!("WriteRegister to SUBGHZ_LSYNCR");
    let _ = spi.write(&[0x0D, 0x07, 0x40, 0x14, 0x24]).await;
    // SetRfFrequency: rffreq = (rf_frequency * 2^25) / f_xtal
    // (868_100_000 * 33_554_432) / 32_000_000 = 910_268_825
    // hex(910_268_825) = 0x36419999
    debug!("SetRfFrequency");
    let _ = spi.write(&[0x86, 0x36, 0x41, 0x99, 0x99]).await;
    // SetPaConfig - +14dBm for SX1262
    debug!("SetPaConfig");
    let _ = spi.write(&[0x95, 0x02, 0x02, 0x00, 0x01]).await;
    // SetTxParams - +22dBm (as written in the docs), 40µs ramp time
    debug!("SetTxParams");
    let _ = spi.write(&[0x8e, 0x16, 0x02]).await;
    // SetModulationParams - sf12, 15.63kHz bandwidth, CR 4/5, ldro off
    debug!("SetModulationParams");
    let _ = spi.write(&[0x8b, 0x0c, 0x01, 0x01, 0x00]).await;
    // SetDioIrqParams - set TxDone and Timeout IRQs
    debug!("SetDioIrqParams");
    let _ = spi.write(&[0x08,
        0b00000000, 0b00000001,  // IrqMask:  bit 0 (TxDone)
        0b00000000, 0b00000001,  // DIO1Mask: same as IrqMask
        0b00000000, 0b00000000,  // DIO2Mask: none
        0b00000000, 0b00000000,  // DIO3Mask: none
    ]).await;
    // Enable RF switch before tx
    let _ctrl1 = Output::new(p.PC4, Level::High, Speed::High); // TX
    let _ctrl2 = Output::new(p.PC5, Level::Low, Speed::High);  // RX
    let _ctrl3 = Output::new(p.PC3, Level::High, Speed::High); // EN
    // SetTx - no timeout
    debug!("SetTx - transmitting now");
    let _ = spi.write(&[0x83, 0x00, 0x00, 0x00]).await;
    // GetIrqStatus loop to poll when the command finishes
    loop {
        debug!("GetIrqStatus");
        // Send 0x12 opcode + 4 nops (extra NOP because SPI is full-duplex, response is shifted)
        let mut buf = [0x12, 0x00, 0x00, 0x00, 0x00];
        let _ = spi.transfer_in_place(&mut buf).await;
        trace!("IRQ raw: {:x}", buf);

        // buf[0] = garbage, buf[1] = status, buf[2] = NOP, buf[3-4] = IrqStatus
        if buf[3] & 0x01 != 0 {
            debug!("GetIrqStatus - tx done");
            break;
        }

        Timer::after_millis(1).await;
    }
    // ClearIrqStatus - all IRQs
    debug!("ClearIrqStatus");
    let _ = spi.write(&[0x02, 0xff, 0xff]).await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
