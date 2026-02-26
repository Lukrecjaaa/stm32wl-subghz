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
use embassy_time::{Duration, Timer};
use stm32wle5jc_radio::{
    modulations::
        bpsk::{Bitrate, BpskConfig, BpskRadio}
    ,
    radio::{PaSelection, Radio},
    spi::SubGhzSpiDevice,
    traits::{Configure, Transmit},
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

    let mut bpsk = BpskRadio::new(&mut radio);
    bpsk.configure(&BpskConfig {
        frequency: 868_100_000,
        bitrate: Bitrate::Bps600,
        pa: PaSelection::HighPower,
        power_dbm: 17,
        ..Default::default()
    })
    .await
    .unwrap();

    info!("sending stuffs");
    match bpsk.tx(b"hiiiiiII!").await {
        Ok(_) => info!("yay :3"),
        Err(e) => error!("tx error: {:?}", e),
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
