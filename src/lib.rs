#![no_std]
#![warn(missing_docs)]

//! TODO add docs

mod regs;
pub use regs::*;

/// It's possible to order devices with other two-bit addresses, but this is the default one.
const SPI_DEVICE_ADDRESS: u8 = 0x01 << 6;

/// 16bit MCP3461/MCP3462/MCP3464/ or 24bit MCP3561/MCP3562/MCP3564/ Analog Digital Converter
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

    /// TODO add docs
    pub fn init(&mut self) -> Result<(), SPI::Error> {
        // use internal clock
        let config0 = Config0::new().with_clk_sel(ClkSel::InternalClock);
        self.write_register_8bit(config0)?;
        // disable en_stp
        let irq = Irq::new().with_en_fastcmd(true);
        self.write_register_8bit(irq)?;
        // be ready for conversions
        self.fast_command(FastCommand::Standby)?;
        Ok(())
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

    #[cfg(any(feature = "mcp3561", feature = "mcp3562", feature = "mcp3564"))]
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

    #[cfg(any(feature = "mcp3461", feature = "mcp3462", feature = "mcp3464"))]
    /// Read a conversion result if [`DataFormat::Format16Default`] is configured
    pub fn read_16_bit_adc_data(&mut self) -> Result<i16, SPI::Error> {
        let mut buf = [RegisterAddress::ADCDATA.into_single_read(), 0x00, 0x00];
        self.spi.transfer_in_place(&mut buf)?;
        Ok(i16::from_be_bytes([buf[1], buf[2]]))
    }

    /// Write a 8bit wide register
    #[inline(always)]
    pub fn write_register_8bit(&mut self, reg: impl Into<Register8Bit>) -> Result<(), SPI::Error> {
        self._write_register_8bit(reg.into())
    }
    fn _write_register_8bit(&mut self, reg: Register8Bit) -> Result<(), SPI::Error> {
        let (addr, value) = reg.addr_value();
        let mut buf = [addr.into_incremental_write(), value];
        self.spi.transfer_in_place(&mut buf)?;
        self.status_byte = StatusByte(buf[0]);
        Ok(())
    }

    /// Read a 8bit wide register
    pub fn read_register_8bit(&mut self, reg: RegisterAddress) -> Result<u8, SPI::Error> {
        let mut buf = [reg.into_single_read(), 0x00];
        self.spi.transfer_in_place(&mut buf)?;
        self.status_byte = StatusByte(buf[0]);
        Ok(buf[1])
    }
}

/// Helper struct for convertion an adc reading to a voltage.
#[derive(Copy, Clone, Debug)]
pub struct ToVoltageConverter24bit(f32);

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

/// Helper struct for convertion an adc reading to a voltage.
#[derive(Copy, Clone, Debug)]
pub struct ToVoltageConverter16bit(f32);

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
    /// ADC Conversion Start/Restart Fast Command (overwrites ADC_MODE[1:0] = 11)
    ConversionStart = 0b1010 << 2,
    /// ADC Standby Mode Fast Command (overwrites ADC_MODE[1:0] = 10)
    Standby = 0b1011 << 2,
    /// ADC Shutdown Mode Fast Command (overwrites ADC_MODE[1:0] = 00)
    Shutdown = 0b1100 << 2,
    /// Full Shutdown Mode Fast Command (overwrites CONFIG0[7:0] = 0x00)
    FullShutdown = 0b1101 << 2,
    /// Device Full Reset Fast Command (resets the entire register map to default value)
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
    ///  4/24/32 R Latest A/D conversion data output value (24 or 32 bits depending on
    /// DATA_FORMAT[1:0]) or modulator output stream (4-bit wide) in MDAT
    /// Output mode
    ADCDATA = 0x0 << 2,
    ///  8 R/W ADC Operating mode, Master Clock mode and Input Bias Current
    /// Source mode
    CONFIG0 = 0x1 << 2,
    ///  8 R/W Prescale and OSR settings
    CONFIG1 = 0x2 << 2,
    /// 8 R/W ADC boost and gain settings, auto-zeroing settings for analog
    /// multiplexer, voltage reference and ADC
    CONFIG2 = 0x3 << 2,
    ///  8 R/W Conversion mode, data and CRC format settings; enable for CRC on
    /// communications, enable for digital offset and gain error calibrations
    CONFIG3 = 0x4 << 2,
    /// 8 R/W IRQ Status bits and IRQ mode settings; enable for Fast commands and
    /// for conversion start pulse
    IRQ = 0x5 << 2,
    ///  8 R/W Analog multiplexer input selection (MUX mode only)
    MUX = 0x6 << 2,
    ///  24 R/W SCAN mode settings
    SCAN = 0x7 << 2,
    ///  24 R/W Delay value for TIMER between SCAN cycles
    TIMER = 0x8 << 2,
    ///  24 R/W ADC digital offset calibration value
    OFFSETCAL = 0x9 << 2,
    ///  24 R/W ADC digital gain calibration value
    GAINCAL = 0xA << 2,
    ///  8 R/W Password value for SPI Write mode locking
    LOCK = 0xD << 2,
    ///  16 R CRC checksum for device configuration
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
