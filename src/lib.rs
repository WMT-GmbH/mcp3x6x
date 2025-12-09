#![no_std]
#![warn(missing_docs)]

//! `no_std` library for the MCP3x6x(R) family of analog digital converters.
//!
//! Supports:
//! * MCP3461
//! * MCP3462
//! * MCP3464
//! * MCP3561
//! * MCP3562
//! * MCP3564
//! * MCP3461R
//! * MCP3462R
//! * MCP3464R
//! * MCP3465R
//! * MCP3561R
//! * MCP3562R
//! * MCP3564R
//! * MCP3565R
//!
//! # Features:
//! * Activate one of the device features to enable support for the corresponding device.
//! * `defmt` feature implements [`defmt::Format`] for all registers.
//!
//! # Basic usage:
//! ```
//! use mcp3x6x::{ClkSel, Config0, FastCommand, Irq, MCP3x6x, ToVoltageConverter24bit};
//!
//! // use 3.3V as Vref+ and 0V as Vref-
//! const TO_VOLT: ToVoltageConverter24bit = ToVoltageConverter24bit::new(3.3, 0.0, mcp3x6x::Gain::X1);
//!
//! # let spi = mcp3x6x::doctesthelper::NoOpSPI;
//! # let irq_pin = mcp3x6x::doctesthelper::MockInput;
//! // spi is a struct implementing embedded_hal::spi::SpiDevice.
//! // irq is an input pin attached to the IRQ pin of the ADC.
//!
//! let mut adc = MCP3x6x::new(spi);
//!
//! // use internal clock
//! let config0 = Config0::new().with_clk_sel(ClkSel::InternalClock);
//! adc.write_register(config0).unwrap();
//!
//! // disable en_stp
//! let irq = Irq::new().with_en_fastcmd(true);
//! adc.write_register(irq).unwrap();
//!
//! // be ready for conversions
//! adc.fast_command(FastCommand::Standby).unwrap();
//!
//! loop {
//!     adc.fast_command(FastCommand::ConversionStart);
//!     while irq_pin.is_high() {}
//!     let sample = adc.read_24_bit_adc_data().unwrap();
//!     let voltage = TO_VOLT.to_volt(sample);
//!     # break
//! }

mod regs;

use embedded_hal::spi::Operation;
pub use regs::*;

/// It's possible to order devices with other two-bit addresses, but this is the default one.
const SPI_DEVICE_ADDRESS: u8 = 0x01 << 6;

/// MCP3x6x(R) Analog Digital Converter
pub struct MCP3x6x<SPI> {
    spi: SPI,
    status_byte: StatusByte,
}

impl<SPI: embedded_hal::spi::SpiDevice> MCP3x6x<SPI> {
    /// Creat the driver
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            status_byte: StatusByte(0),
        }
    }

    /// Get the last received status byte.
    /// Invalid if no SPI communication happened yet.
    pub fn status_byte(&self) -> StatusByte {
        self.status_byte
    }

    /// Send a [`FastCommand`]
    pub fn fast_command(&mut self, cmd: FastCommand) -> Result<StatusByte, SPI::Error> {
        let mut buf = [cmd.into_command_byte()];
        self.spi.transfer_in_place(&mut buf)?;
        self.status_byte = StatusByte(buf[0]);
        Ok(self.status_byte)
    }

    #[cfg(feature = "__24_bit")]
    /// Read a conversion result if [`DataFormat::Format24Default`] is configured
    pub fn read_24_bit_adc_data(&mut self) -> Result<i32, SPI::Error> {
        let mut buf = [
            RegisterAddress::ADCDATA.into_single_read(),
            0x00,
            0x00,
            0x00,
        ];
        self.spi.transfer_in_place(&mut buf)?;
        let mut value = (buf[1] as i32) << 16 | (buf[2] as i32) << 8 | buf[3] as i32;
        if value & 0x0080_0000 != 0 {
            value |= 0xFF00_0000u32 as i32;
        }
        Ok(value)
    }

    #[cfg(feature = "__16_bit")]
    /// Read a conversion result if [`DataFormat::Format16Default`] is configured
    pub fn read_16_bit_adc_data(&mut self) -> Result<i16, SPI::Error> {
        let mut buf = [RegisterAddress::ADCDATA.into_single_read(), 0x00, 0x00];
        self.spi.transfer_in_place(&mut buf)?;
        Ok(i16::from_be_bytes([buf[1], buf[2]]))
    }

    /// Write a register.
    pub fn write_register<R: WriteableRegister>(&mut self, reg: R) -> Result<(), SPI::Error> {
        let mut first_byte = [R::ADDRESS.into_incremental_write()];
        self.spi.transaction(&mut [
            Operation::TransferInPlace(&mut first_byte),
            Operation::Write(reg.bytes()),
        ])?;
        self.status_byte = StatusByte(first_byte[0]);
        Ok(())
    }

    /// Read a register.
    pub fn read_register<R: Register>(&mut self) -> Result<R, SPI::Error> {
        let mut first_byte = [R::ADDRESS.into_single_read()];
        let mut buf = R::Bytes::default();
        self.spi.transaction(&mut [
            Operation::TransferInPlace(&mut first_byte),
            Operation::Read(buf.as_mut()),
        ])?;
        self.status_byte = StatusByte(first_byte[0]);
        Ok(R::new(buf))
    }
}

#[cfg(feature = "__24_bit")]
/// Helper struct for convertion an adc reading to a voltage.
#[derive(Copy, Clone, Debug)]
pub struct ToVoltageConverter24bit(f32);

#[cfg(feature = "__24_bit")]
impl ToVoltageConverter24bit {
    /// Call this in `const`
    // Implements EQUATION 5-5 from the datasheet
    pub const fn new(vref_p: f64, vref_n: f64, gain: Gain) -> Self {
        let factor = (vref_p - vref_n) / ((1 << 23) as f64) / gain.amplification() as f64;
        ToVoltageConverter24bit(factor as f32)
    }

    /// Convert an adc reading to a voltage.
    pub fn to_volt(self, output: i32) -> f32 {
        output as f32 * self.0
    }
}

#[cfg(feature = "__16_bit")]
/// Helper struct for convertion an adc reading to a voltage.
#[derive(Copy, Clone, Debug)]
pub struct ToVoltageConverter16bit(f32);

#[cfg(feature = "__16_bit")]
impl ToVoltageConverter16bit {
    /// Call this in `const`
    // Implements EQUATION 5-5 from the datasheet
    pub const fn new(vref_p: f64, vref_n: f64, gain: Gain) -> Self {
        let factor = (vref_p - vref_n) / ((1 << 15) as f64) / gain.amplification() as f64;
        ToVoltageConverter16bit(factor as f32)
    }

    /// Convert an adc reading to a voltage.
    pub fn to_volt(self, output: i16) -> f32 {
        output as f32 * self.0
    }
}

/// Fast command
#[repr(u8)]
pub enum FastCommand {
    /// ADC Conversion Start/Restart Fast Command
    ConversionStart = 0b1010 << 2,
    /// ADC Standby Mode Fast Command
    Standby = 0b1011 << 2,
    /// ADC Shutdown Mode Fast Command
    Shutdown = 0b1100 << 2,
    /// Full Shutdown Mode Fast Command
    FullShutdown = 0b1101 << 2,
    /// Device Full Reset Fast Command
    Reset = 0b1110 << 2,
}

impl FastCommand {
    fn into_command_byte(self) -> u8 {
        SPI_DEVICE_ADDRESS | self as u8
    }
}

/// The three Interrupt Status bits are independent of the two other interrupt
/// mechanisms (IRQ pin and IRQ register) and are cleared each time the
/// STATUS byte is fully clocked.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusByte(pub u8);

impl StatusByte {
    /// ADC data ready interrupt status
    pub fn dr_status(self) -> bool {
        self.0 & (1 << 2) == 0
    }
    /// CRC checksum error on the register map interrupt status
    pub fn crccfg_status(self) -> bool {
        self.0 & (1 << 1) == 0
    }
    /// POR interrupt status
    pub fn por_status(self) -> bool {
        self.0 & (1 << 0) == 0
    }
}

/// Available registers
#[repr(u8)]
pub enum RegisterAddress {
    /// Latest A/D conversion data output value or modulator output stream in MDAT Output mode
    ADCDATA = 0x0 << 2,
    /// ADC Operating mode, Master Clock mode and Input Bias Current
    /// Source mode
    CONFIG0 = 0x1 << 2,
    /// Prescale and OSR settings
    CONFIG1 = 0x2 << 2,
    /// ADC boost and gain settings, auto-zeroing settings for analog
    /// multiplexer, voltage reference and ADC
    CONFIG2 = 0x3 << 2,
    /// Conversion mode, data and CRC format settings; enable for CRC on
    /// communications, enable for digital offset and gain error calibrations
    CONFIG3 = 0x4 << 2,
    /// IRQ Status bits and IRQ mode settings; enable for Fast commands and
    /// for conversion start pulse
    IRQ = 0x5 << 2,
    /// Analog multiplexer input selection (MUX mode only)
    MUX = 0x6 << 2,
    /// SCAN mode settings
    SCAN = 0x7 << 2,
    /// Delay value for TIMER between SCAN cycles
    TIMER = 0x8 << 2,
    /// ADC digital offset calibration value
    OFFSETCAL = 0x9 << 2,
    /// ADC digital gain calibration value
    GAINCAL = 0xA << 2,
    /// Password value for SPI Write mode locking
    LOCK = 0xD << 2,
    /// CRC checksum for device configuration
    CRCCFG = 0xF << 2,
}

impl RegisterAddress {
    fn into_single_read(self) -> u8 {
        SPI_DEVICE_ADDRESS | self as u8 | 0b01
    }

    #[allow(unused)]
    fn into_incremental_read(self) -> u8 {
        SPI_DEVICE_ADDRESS | self as u8 | 0b11
    }

    fn into_incremental_write(self) -> u8 {
        SPI_DEVICE_ADDRESS | self as u8 | 0b10
    }
}

#[doc(hidden)]
// FIXME: #[cfg(doctest)] once https://github.com/rust-lang/rust/issues/67295 is fixed.
pub mod doctesthelper;
