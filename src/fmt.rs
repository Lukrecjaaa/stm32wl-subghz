//! Each macro forwards to `defmt` when the `defmt` feature is on, and is a
//! no-op otherwise. The no-op version still references each argument so
//! variables consumed only by logging don't trigger unused warnings

#![allow(unused_macros)]

#[cfg(feature = "defmt")]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => { ::defmt::trace!($s $(, $x)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => { $( let _ = &$x; )* };
}

#[cfg(feature = "defmt")]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => { ::defmt::debug!($s $(, $x)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => { $( let _ = &$x; )* };
}

#[cfg(feature = "defmt")]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => { ::defmt::info!($s $(, $x)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => { $( let _ = &$x; )* };
}

#[cfg(feature = "defmt")]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => { ::defmt::warn!($s $(, $x)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => { $( let _ = &$x; )* };
}

#[cfg(feature = "defmt")]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => { ::defmt::error!($s $(, $x)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => { $( let _ = &$x; )* };
}
