//! used for doc tests

use core::convert::Infallible;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

pub struct NoOpSPI;

impl SpiDevice for NoOpSPI {
    fn transaction(&mut self, _: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl ErrorType for NoOpSPI {
    type Error = Infallible;
}

pub struct MockInput;

impl MockInput {
    pub fn is_high(&self) -> bool {
        false
    }
    pub fn is_low(&self) -> bool {
        true
    }
}
