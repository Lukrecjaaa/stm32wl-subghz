#![cfg_attr(feature = "hal", no_std)]
#![allow(async_fn_in_trait)]

pub mod modulations;

#[cfg(feature = "hal")]
pub mod error;
#[cfg(feature = "hal")]
pub mod radio;
#[cfg(feature = "hal")]
pub mod spi;
#[cfg(feature = "hal")]
pub mod traits;

#[cfg(feature = "hal")]
pub use error::RadioError;
#[cfg(feature = "hal")]
pub use radio::{PaSelection, Radio};
#[cfg(feature = "hal")]
pub use spi::SubGhzSpiDevice;
#[cfg(feature = "hal")]
pub use traits::{Configure, Receive, Transmit};
