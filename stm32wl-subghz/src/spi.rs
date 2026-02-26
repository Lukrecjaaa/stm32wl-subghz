use defmt::trace;
use embassy_stm32::pac;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};

/// Wrapper for the sub-GHz SPI device
pub struct SubGhzSpiDevice<T>(pub T);

impl<T: SpiBus> ErrorType for SubGhzSpiDevice<T> {
    type Error = T::Error;
}

/// This works as a translation layer between normal SPI transactions and sub-GHz device SPI
/// transactions. Everything above this layer sees it like a normal SPI device!
impl<T: SpiBus> SpiDevice for SubGhzSpiDevice<T> {
    /// Perform a transaction on the sub-GHz device
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        // Pull NSS low to allow SPI comms
        pac::PWR.subghzspicr().modify(|w| w.set_nss(false));
        trace!("NSS low");
        for operation in operations {
            match operation {
                Operation::Read(buf) => {
                    self.0.read(buf).await?;
                    trace!("Read {:x}", buf);
                }
                Operation::Write(buf) => {
                    self.0.write(buf).await?;
                    trace!("Wrote {:x}", buf);
                }
                Operation::Transfer(read, write) => {
                    self.0.transfer(read, write).await?;
                    trace!("Read {:x} wrote {:x}", read, write);
                }
                Operation::TransferInPlace(buf) => {
                    self.0.transfer_in_place(buf).await?;
                    trace!("Read+wrote {:x}", buf);
                }
                Operation::DelayNs(_) => {}
            }
        }
        // Pull NSS high
        pac::PWR.subghzspicr().modify(|w| w.set_nss(true));
        trace!("NSS high");
        // Small delay for the radio to assert BUSY after NSS goes high
        cortex_m::asm::delay(500);
        // Poll BUSY flag until it's done
        while pac::PWR.sr2().read().rfbusys() {}
        trace!("BUSY flag clear");
        Ok(())
    }
}
