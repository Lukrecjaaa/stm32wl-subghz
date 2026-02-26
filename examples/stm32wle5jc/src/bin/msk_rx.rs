#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    gpio::{Level, Output, Speed},
    rcc::{MSIRange, Sysclk, mux},
    spi::Spi,
};
use stm32wl_subghz::{
    Configure, PaSelection, Radio, RadioError, Receive, SubGhzSpiDevice,
    modulations::msk::{Bitrate, MskConfig, MskRadio, PulseShape},
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

    let mut msk = MskRadio::new(&mut radio);
    msk.configure(&MskConfig {
        frequency: 868_100_000,
        bitrate: Bitrate::Custom(600),
        pulse_shape: PulseShape::GaussianBt05,
        pa: PaSelection::HighPower,
        power_dbm: 22,
        ..Default::default()
    })
    .await
    .unwrap();

    info!("waiting for msk stuffs...");
    let mut buf = [0u8; 255];
    match msk.rx(&mut buf, 5_000).await {
        Ok(len) => info!("yay :3 got {} bytes: {:x}", len, &buf[..len]),
        Err(RadioError::Timeout) => warn!("nobody is talking to me :<"),
        Err(e) => error!("rx error: {:?}", e),
    }
}
