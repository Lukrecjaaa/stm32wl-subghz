# stm32wl-subghz examples for STM32WLE5JC

Simple rx/tx examples targeting the STM32WLE5JC chip (tested on LoRa E5-mini board).

## Building and running
```sh
cargo build --release --bin <name> # eg. bpsk_tx
```

```sh
cargo run --release --bin <name> # eg. bpsk_tx
```

The run command should automatically use `probe-rs` to flash the compiled binary on the microcontroller.

Adjust the board name and targets in `./cargo/config.toml`, `./Cargo.toml` (`embassy-stm32` feature) and `./rust-toolchain.toml` if your board differs.
