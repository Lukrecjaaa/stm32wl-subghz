# stm32wl-subghz

[![Latest Version]][crates.io]

Sub-GHZ SPI device radio driver for STM32WL-series microcontrollers, currently supporting LoRa and BPSK.

## Usage

Add the dependency with `default-features = false` and enable your chip's feature:

```toml
[dependencies]
stm32wl-subghz = { version = "0.1.0", default-features = false, features = ["stm32wle5jc"] }
```

A chip feature **must** be enabled. See `Cargo.toml` for the full list of supported chips (all STM32WLE single-core and STM32WL5x dual-core variants).

> The default feature enables `stm32wle5jc` for development convenience. Always use `default-features = false` in your project to avoid feature conflicts.

## Crates
- [`stm32wl-subghz`](.) - the driver library
- [`examples/stm32wle5jc`](./examples/stm32wle5jc/) - usage examples for the STM32WLE5JC-based boards

## License
MIT

[Latest Version]: https://img.shields.io/crates/v/stm32wl-subghz.svg
[crates.io]: https://crates.io/crates/stm32wl-subghz
