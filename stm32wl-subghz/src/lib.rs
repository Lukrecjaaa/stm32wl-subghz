#![no_std]
#![allow(async_fn_in_trait)]

pub mod error;
pub mod modulations;
pub mod radio;
pub mod spi;
pub mod traits;

pub use error::RadioError;
pub use radio::{PaSelection, Radio};
pub use spi::SubGhzSpiDevice;
pub use traits::{Configure, Receive, Transmit};
