use crate::error::RadioError;

/// Can configure the radio
pub trait Configure {
    /// Each modulation has its own `Config` struct
    type Config;

    async fn configure(&mut self, config: &Self::Config) -> Result<(), RadioError>;
}

/// Can send data
pub trait Transmit {
    async fn tx(&mut self, data: &[u8]) -> Result<(), RadioError>;
}

/// Can receive data
pub trait Receive {
    /// Returns the number of bytes received
    async fn rx(&mut self, buf: &mut [u8], timeout_ms: u32) -> Result<usize, RadioError>;
}
