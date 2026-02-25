#![no_std]
#![no_main]

use defmt::{debug, trace};
use embassy_executor::Spawner;
use embassy_stm32::{Config, pac, rcc::{MSIRange, Sysclk, mux}, spi::Spi};
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
    trace!("RFRST high");
    Timer::after_millis(1).await;
    pac::RCC.csr().modify(|w| w.set_rfrst(false));
    trace!("RFRST low");
    Timer::after_millis(1).await;
    while pac::PWR.sr2().read().rfbusys() {}
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

    reset_radio().await;

    let mut spi = SubGhzSpiDevice(Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2));
    debug!("Writing SetStandby");
    let _ = spi.write(&[0x80, 0x00]).await;
    debug!("Radio in standby!");

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
