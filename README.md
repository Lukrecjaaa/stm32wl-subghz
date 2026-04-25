# stm32wl-subghz

[![Latest Version]][crates.io]

An async `no_std` driver for the sub-GHz radio embedded in STM32WL-series microcontrollers (the SX126x IP block). It supports LoRa, (G)FSK, MSK and BPSK modulations through a small set of modulation-specific wrappers built on top of a common low-level radio core.

> [!WARNING]
> This crate is experimental and shouldn't be used in production. Don't transmit on ISM bands without understanding your local duty cycle limits, ERP restrictions and other regulations! This crate does NOT validate airtime and has no way to adhere to regional regulations. Yet!

## What it does

The integrated sub-GHz radio on STM32WL chips is a SX1261/SX1262-based radio connected to the core through a private SPI bus. This crate wraps that bus, talks to the radio over the documented opcode interface (RM0461), and exposes a friendly Rust API on top.

The architecture has a few layers:

```
Modulation-specific radio (eg. `LoraRadio`)
  |
  | user-specified configs
  v
`Radio` for translating everything to raw op-codes, addresses, etc
  |
  | low-level SPI commands
  v
`SubGhzSpiDevice`
  |
  | internal SPI comms
  v
`SUBGHZSPI` peripheral
```

Each modulation wrapper borrows a `&mut Radio`, so the borrow checker statically prevents two modulations from being active at the same time. Configuration is done through a per-modulation `Config` struct passed to the `Configure` trait, and TX/RX is done through the `Transmit` and `Receive` traits.

### Modulation notes

- **LoRa**: spreading factor, bandwidth, coding rate, header type, CRC, IQ inversion. The most commonly-used modulation type on these kind of radios.
- **FSK**: bitrate, frequency deviation, RX bandwidth, sync word, CRC, whitening, packet length type. All parameters are exposed so it's easy to tune them.
- **MSK**: actually implemented on top of FSK with the modulation index pinned to 0.5 (so `Fdev = bitrate / 4`). The chip technically lists `PacketType::Msk = 0x03`, but in practice that mode is barely documented and produces an unmodulated carrier, so the driver routes MSK through the FSK packet engine instead. Both TX and RX work.
- **BPSK**: TX only (the chip has no BPSK packet engine for RX). Because the modem only handles raw bits, framing has to be done in software. The driver provides a `BpskPacket` enum with two variants: `Raw` for unframed transmission and `Framing { preamble_len, sync_word, sync_word_len, crc_type, whitening, whitening_seed }` for proper packets that can survive demodulation. A 1-byte length field is always written before the payload in `Framing` mode. CRC-8 (`0x07`), CRC-16 CCITT (`0x1021`) and CCITT whitening (`x^9 + x^4 + 1` LFSR) are implemented.

## Usage

Add the crate with `default-features = false` and pick the feature that matches your chip:

```toml
[dependencies]
stm32wl-subghz = { version = "0.1.0", default-features = false, features = ["stm32wle5jc"] }
```

A chip feature **must** be enabled. See [`Cargo.toml`](./Cargo.toml) for the full list, which covers all single-core STM32WLEx variants and both cores of the dual-core STM32WL5x family.

> [!WARNING]
> All the other variants are available, but only the `stm32wle5jc` was tested by me as I only had access to this particular board. It might or might not work on other boards.

Available feature flags:
- `stm32wlXXX`: enables the appropriate `embassy-stm32` chip feature and the `hal` feature
- `hal`: pulls in the embedded stack (embassy-rs) and its dependencies. Transitively enabled by every chip feature, so you don't need to list it explicitly. Leave all chip features off if you only want the modulation-related types that work in `std`-enabled targets (see [BPSK decoding](#bpsk-rx-and-decoding) below).
- `defmt`: derives `defmt::Format` for public types and enables logging via `defmt`

> The crate's default feature enables `stm32wle5jc` for development convenience. Always use `default-features = false` in your project so feature flags don't conflict.

## Examples

The [`examples/stm32wle5jc`](./examples/stm32wle5jc/) workspace member contains runnable examples for all four modulations:

| File | What it does |
| --- | --- |
| `lora_tx.rs` / `lora_rx.rs` | Configure LoRa and send/receive a payload |
| `fsk_tx.rs` / `fsk_rx.rs` | Same, with variable-length FSK packets |
| `msk_tx.rs` / `msk_rx.rs` | MSK via FSK with `Fdev = bitrate / 4` |
| `bpsk_tx.rs` | Framed BPSK transmission (preamble + sync + len + CRC + whitening) |

The examples target the [LoRa-E5 mini](https://wiki.seeedstudio.com/LoRa_E5_mini/) board, which exposes the RF switch on `PC3` (EN), `PC4` (TX) and `PC5` (RX). If your board wires the switch differently, adjust those three pins in the example's `main`.

A typical example reads (trimmed):

```rust
let spi = SubGhzSpiDevice(Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2));
let rf_tx = Output::new(p.PC4, Level::Low, Speed::High);
let rf_rx = Output::new(p.PC5, Level::Low, Speed::High);
let rf_en = Output::new(p.PC3, Level::Low, Speed::High);
let mut radio = Radio::new(spi, rf_tx, rf_rx, rf_en);
radio.init().await.unwrap();

let mut lora = LoraRadio::new(&mut radio);
lora.configure(&LoraConfig {
    frequency: 868_100_000,
    sf: SpreadingFactor::SF9,
    bw: Bandwidth::Bw20_83kHz,
    pa: PaSelection::HighPower,
    power_dbm: 22,
    ..Default::default()
}).await.unwrap();

lora.tx(b"hiiiii :3").await.unwrap();
```

If you've never flashed an STM32WL board before and the factory firmware is locked behind RDP level 1 (which it usually is on the LoRa-E5 mini), [`stm32wl-unlock`](https://github.com/newAM/stm32wl-unlock) will clear it for you without needing STM32Cube.

### BPSK RX and decoding

There is no `bpsk_rx` embedded example, because the chip's BPSK modem is transmit-only. To actually receive a BPSK transmission you need an external receiver: an SDR plus a demodulator that recovers carrier phase and symbol timing. The full pipeline that this crate has been validated against looks like:

1. Capture raw IQ samples with an SDR (e.g. `rtl_sdr -f 868100000 -s 250000 -g 40 capture.raw`).
2. Demodulate with GNU Radio: low-pass filter and decimate, AGC, Costas loop for phase recovery, symbol sync for timing recovery, slice to bits, pack into bytes, write to a file. A working flowchart is described step by step in [part 5 of the blog series](https://lusia.moe/posts/stm32-part-5/).
3. Decode the framed bytes back into the original payload using `BpskPacket::decode`.

The decode step is provided by this crate, gated on the `hal` feature being **disabled** (so it's available on a host machine running `std`):

```toml
[dependencies]
stm32wl-subghz = { version = "0.1.0", default-features = false }
```

```rust
use std::{env, fs};

use stm32wl_subghz::modulations::bpsk::BpskPacket;

fn main() {
    let help_str = "Usage: bpsk-rx <file.bin> [options]\nOptions:\n--show-all: shows packets with invalid CRC";
    let path = env::args().nth(1).expect(help_str);
    let show_all = matches!(env::args().nth(2).unwrap_or_default().as_str(), "--show-all");

    let data = fs::read(&path).unwrap();

    let packet = BpskPacket::default();
    let decode_results = packet.decode(&data);

    println!("Found {} packets!!", decode_results.len());

    for result in decode_results {
        if !result.crc_valid && !show_all {
            continue;
        }
        println!("offset {}\ninverted {}\ncrc valid {}\npayload hex {:x?}\npayload utf-8 {:?}\n\n",
            result.bit_offset,
            result.inverted,
            result.crc_valid,
            result.payload,
            String::from_utf8_lossy(&result.payload)
        );
    }
}
```

The decoder slides a sync-word matcher bit-by-bit across the demodulated stream, allows up to 2 bit errors, transparently handles the 180 degree phase ambiguity that BPSK demodulation always has, reverses the whitening and validates the CRC. Anything that survives all four checks is returned with `crc_valid: true`.

## Roadmap

Things that work today:
- LoRa, FSK and MSK transmission and reception, validated board-to-board
- BPSK transmission with software framing (preamble, sync word, length, CRC-8/CRC-16, CCITT whitening)
- Off-chip BPSK reception via SDR + GNU Radio + the bundled `decode` function

Things that are still TODO:
- Continuous RX mode (currently every receive is single-shot with a timeout)
- Payload sizes larger than 255 bytes (the chip's data buffer supports it via segmentation, see RM0461 section 4.6)
- compliance with airtime limits and other legal stuff
- Hardware testing on STM32WL chips other than STM32WLE5JC
- More end-to-end examples: a TUN/TAP bridge, telemetry from a stratospheric balloon, anything fun

## Crates

- [`stm32wl-subghz`](.): the driver library
- [`examples/stm32wle5jc`](./examples/stm32wle5jc/): runnable examples for STM32WLE5JC-based boards

## References

- [RM0461](https://www.st.com/resource/en/reference_manual/rm0461-stm32wlex-advanced-armbased-32bit-mcus-with-subghz-radio-solution-stmicroelectronics.pdf): the STM32WLEx reference manual, the single most useful document for this whole project
- [STM32WLE5JC datasheet](https://www.st.com/resource/en/datasheet/stm32wle5jc.pdf)
- [SX1261/SX1262 datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)
- [`lora-rs`](https://github.com/lora-rs/lora-rs): another sub-GHz driver for STM32WL, LoRa-only, was a useful reference for the SPI translation layer
- [`stm32wl-unlock`](https://github.com/newAM/stm32wl-unlock): clears RDP level 1 on factory-locked boards
- [Blog series on building this driver](https://lusia.moe/tags/stm32wle5jc/) (parts 1 to 5): the long-form story of how the crate came together, including the painful parts

## License

MIT

[Latest Version]: https://img.shields.io/crates/v/stm32wl-subghz.svg
[crates.io]: https://crates.io/crates/stm32wl-subghz
