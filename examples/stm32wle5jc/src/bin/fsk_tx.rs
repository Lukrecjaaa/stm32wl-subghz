#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    gpio::{Level, Output, Speed},
    rcc::{MSIRange, Sysclk, mux},
    spi::Spi,
};
use stm32wl_subghz::{
    Configure, PaSelection, Radio, SubGhzSpiDevice, Transmit,
    modulations::fsk::{Bitrate, FreqDev, FskConfig, FskRadio},
};
use {defmt_rtt as _, panic_probe as _};

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

    let spi = SubGhzSpiDevice(Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2));
    let rf_tx = Output::new(p.PC4, Level::Low, Speed::High);
    let rf_rx = Output::new(p.PC5, Level::Low, Speed::High);
    let rf_en = Output::new(p.PC3, Level::Low, Speed::High);
    let mut radio = Radio::new(spi, rf_tx, rf_rx, rf_en);
    radio.init().await.unwrap();

    let mut fsk = FskRadio::new(&mut radio);
    fsk.configure(&FskConfig {
        frequency: 868_100_000,
        bitrate: Bitrate::Custom(600),
        fdev: FreqDev::Hz(380),
        pa: PaSelection::HighPower,
        power_dbm: 22,
        ..Default::default()
    })
    .await
    .unwrap();

    info!("sending fsk stuffs");
    match fsk.tx(b"hiiiii hello :3 :3 :3 this is a looooooooooooooooong text! very long :> and cute! :3 :3 :3 :3 :3 :3 :3 :3 :3 :3 :3 :3 :3 :3 ummmm urghhh awwwooooooo woof wooooof woof").await {
        Ok(_) => info!("yay tx done :3"),
        Err(e) => error!("tx error: {:?}", e),
    }
}
