#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RadioError {
    /// SPI comms failed
    Spi,
    /// Radio busy (polling rfbusys)
    Busy,
    /// Timeout of tx or rx
    Timeout,
    /// Rx packet had bad CRC
    CrcInvalid,
    /// Header was invalid
    HeaderInvalid,
    /// Payload too large
    PayloadTooLarge,
    /// Invalid configuration (e.g. power out of range for selected PA)
    InvalidConfig,
    /// Command not allowed in the current radio state
    InvalidState,
}
