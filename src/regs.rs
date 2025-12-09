use crate::RegisterAddress;
use modular_bitfield::prelude::*;

#[cfg(not(feature = "__has_voltage_reference"))]
/// ADC Operating mode, Master Clock mode and Input Bias Current Source mode
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config0 {
    /// ADC Operating Mode Selection
    pub adc_mode: AdcMode,
    /// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-)
    pub cs_sel: CsSel,
    /// Clock Selection
    pub clk_sel: ClkSel,
    #[skip]
    __: B2,
}

#[cfg(feature = "__has_voltage_reference")]
/// ADC Operating mode, Master Clock mode and Input Bias Current Source mode
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config0 {
    /// ADC Operating Mode Selection
    pub adc_mode: AdcMode,
    /// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-)
    pub cs_sel: CsSel,
    /// Clock Selection
    pub clk_sel: ClkSel,
    #[skip]
    __: B1,
    /// Internal Voltage Reference Bit
    pub vref_sel: bool,
}

/// Clock Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum ClkSel {
    /// Internal clock is selected and AMCLK is present on the analog master clock output pin
    InternalClockAndAMCLK = 0b11,
    /// Internal clock is selected and no clock output is present on the CLK pin
    InternalClock = 0b10,
    /// External digital clock (default)
    ExternalDigitalClock = 0b00,
}

/// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum CsSel {
    /// 15 µA is applied to the ADC inputs
    Cs15uA = 0b11,
    /// 3.7 µA is applied to the ADC inputs
    Cs3_7uA = 0b10,
    /// 0.9 µA is applied to the ADC inputs
    Cs0_9uA = 0b01,
    /// No current source is applied to the ADC inputs (default)
    CsOff = 0b00,
}

/// ADC Operating Mode Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum AdcMode {
    /// 11 = ADC Conversion mode
    Conversion = 0b11,
    /// 10 = ADC Standby mode
    Standby = 0b10,
    /// 00 = ADC Shutdown mode (default)
    ShutdownDefault = 0b00,
}

/// Prescale and OSR settings
#[bitfield]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub struct Config1 {
    #[skip]
    __: B2,
    /// Oversampling Ratio for Delta-Sigma A/D Conversion
    pub osr: Osr,
    /// Prescaler Value Selection for AMCLK
    pub pre: Pre,
}

/// Prescaler Value Selection for AMCLK
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum Pre {
    /// AMCLK = MCLK/8
    MclkDiv8 = 0b11,
    /// AMCLK = MCLK/4
    MclkDiv4 = 0b10,
    /// AMCLK = MCLK/2
    MclkDiv2 = 0b01,
    /// AMCLK = MCLK (default)
    Mclk = 0b00,
}

/// Oversampling Ratio for Delta-Sigma A/D Conversion
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum Osr {
    /// OSR: 98304
    Osr98304 = 0b1111,
    /// OSR: 81920
    Osr81920 = 0b1110,
    /// OSR: 49152
    Osr49152 = 0b1101,
    /// OSR: 40960
    Osr40960 = 0b1100,
    /// OSR: 24576
    Osr24576 = 0b1011,
    /// OSR: 20480
    Osr20480 = 0b1010,
    /// OSR: 16384
    Osr16384 = 0b1001,
    /// OSR: 8192
    Osr8192 = 0b1000,
    /// OSR: 4096
    Osr4096 = 0b0111,
    /// OSR: 2048
    Osr2048 = 0b0110,
    /// OSR: 1024
    Osr1024 = 0b0101,
    /// OSR: 512
    Osr512 = 0b0100,
    /// OSR: 256 (default)
    Osr256 = 0b0011,
    /// OSR: 128
    Osr128 = 0b0010,
    /// OSR: 64
    Osr64 = 0b0001,
    /// OSR: 32
    Osr32 = 0b0000,
}

#[cfg(not(feature = "__has_voltage_reference"))]
///  ADC boost and gain settings, auto-zeroing settings for analog
/// multiplexer, voltage reference and ADC
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config2 {
    #[skip]
    __: B2,
    /// Auto-Zeroing MUX Setting
    pub az_mux: bool,
    /// ADC Gain Selection
    pub gain: Gain,
    /// ADC Bias Current Selection
    pub boost: Boost,
}

#[cfg(feature = "__has_voltage_reference")]
///  ADC boost and gain settings, auto-zeroing settings for analog
/// multiplexer, voltage reference and ADC
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config2 {
    #[skip]
    __: B1,
    /// Auto-Zeroing Reference Buffer Setting
    pub az_ref: bool,
    /// Auto-Zeroing MUX Setting
    pub az_mux: bool,
    /// ADC Gain Selection
    pub gain: Gain,
    /// ADC Bias Current Selection
    pub boost: Boost,
}

/// ADC Bias Current Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum Boost {
    /// ADC channel has current ×2
    X2 = 0b11,
    /// ADC channel has current ×1 (default)
    X1 = 0b10,
    /// ADC channel has current ×0.66
    X0_66 = 0b01,
    /// ADC channel has current ×0.5
    X0_5 = 0b00,
}

/// ADC Gain Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum Gain {
    /// Gain ×64 (×16 analog, ×4 digital)
    X64 = 0b111,
    /// Gain ×32 (×16 analog, ×2 digital)
    X32 = 0b110,
    /// Gain ×16
    X16 = 0b101,
    /// Gain ×8
    X8 = 0b100,
    /// Gain ×4
    X4 = 0b011,
    /// Gain ×2
    X2 = 0b010,
    /// Gain ×1 (default)
    X1 = 0b001,
    /// Gain ×1/3
    Div3 = 0b000,
}

impl Gain {
    /// Get the amplification factor as a `f32`
    pub const fn amplification(self) -> f32 {
        match self {
            Gain::X64 => 64.0,
            Gain::X32 => 32.0,
            Gain::X16 => 16.0,
            Gain::X8 => 8.0,
            Gain::X4 => 4.0,
            Gain::X2 => 2.0,
            Gain::X1 => 1.0,
            Gain::Div3 => 1.0 / 3.0,
        }
    }
}

/// Conversion mode, data and CRC format settings; enable for CRC on
/// communications, enable for digital offset and gain error calibrations
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config3 {
    /// Enable Digital Gain Calibration (default = false)
    pub en_gaincal: bool,
    /// Enable Digital Offset Calibration (default = false)
    pub en_offcal: bool,
    /// CRC Checksum Selection on Read Communications (default = false)
    pub en_crccom: bool,
    /// CRC Checksum Format Selection on Read Communications
    pub crc_format: CrcFormat,
    /// ADC Output Data Format Selection
    pub data_format: DataFormat,
    /// Conversion Mode Selection
    pub conv_mode: ConvMode,
}

/// CRC Checksum Format Selection on Read Communications
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 1]
pub enum CrcFormat {
    /// 32-bit wide (CRC-16 followed by 16 zeros)
    Wide32 = 1,
    /// 16-bit wide (CRC-16 only) (default)
    Wide16 = 0,
}

/// Conversion Mode Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum ConvMode {
    /// Continuous Conversion mode or continuous conversion cycle in SCAN mode
    Continuous = 0b11,
    /// One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE to ‘10’ (standby) at
    /// the end of the conversion or at the end of the conversion cycle in SCAN mode.
    OneShotStandby = 0b10,
    /// One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE to ‘0x’ (ADC
    /// Shutdown) at the end of the conversion or at the end of the conversion cycle in SCAN mode (default).
    OneShotShutdown = 0b00,
}

#[cfg(feature = "__24_bit")]
/// ADC Output Data Format Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum DataFormat {
    /// 32-bit (25-bit right justified data + Channel ID): CHID + SGN extension (4 bits) + 24-bit ADC
    /// data. It allows overrange with the SGN extension.
    Format32ChId = 0b11,
    ///  32-bit (25-bit right justified data): SGN extension (8-bit) + 24-bit ADC data. It allows overrange with
    /// the SGN extension.
    Format32SgnExt = 0b10,
    /// 32-bit (24-bit left justified data): 24-bit ADC data + 0x00 (8-bit). It does not allow overrange (ADC
    /// code locked to 0xFFFFFF or 0x800000).
    Format32Left = 0b01,
    /// 24-bit (default ADC coding): 24-bit ADC data. It does not allow overrange (ADC code locked to
    /// 0xFFFFFF or 0x800000).
    Format24Default = 0b00,
}

#[cfg(not(feature = "__24_bit"))]
/// ADC Output Data Format Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 2]
pub enum DataFormat {
    /// 32-bit (17-bit right justified data + Channel ID): CHID + SGN extension (4 bits) + 16-bit ADC
    /// data. It allows overrange with the SGN extension.
    Format32ChId = 0b11,
    ///  32-bit (17-bit right justified data): SGN extension (8-bit) + 16-bit ADC data. It allows overrange with
    /// the SGN extension.
    Format32SgnExt = 0b10,
    /// 32-bit (16-bit left justified data): 16-bit ADC data + 0x00 (8-bit). It does not allow overrange (ADC
    /// code locked to 0xFFFFFF or 0x800000).
    Format32Left = 0b01,
    /// 16-bit (default ADC coding): 16-bit ADC data. It does not allow overrange (ADC code locked to
    /// 0xFFFFFF or 0x800000).
    Format16Default = 0b00,
}

/// IRQ Status bits and IRQ mode settings; enable for Fast commands and
/// for conversion start pulse
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Irq {
    /// Enable Conversion Start Interrupt Output
    pub en_stp: bool,
    /// Enable Fast Commands
    pub en_fastcmd: bool,
    /// IRQ Pin Inactive State Selection
    /// true: The Inactive state is logic high (does not require a pull-up resistor to DVDD)
    /// false: The Inactive state is high-Z (requires a pull-up resistor to DVDD) (default)
    pub push_pull: bool,
    /// IRQ/MDAT Selection
    /// true: MDAT output is selected. Only POR and CRC interrupts can be present on this pin and take priority over the MDAT output.
    /// false: IRQ output is selected. All interrupts can appear on the IRQ/MDAT pin. (default)
    pub mdat: bool,
    #[skip(setters)]
    /// Power-On Reset Status Flag
    pub por_status: bool,
    #[skip(setters)]
    /// CRC Error Status Flag for Configuration Registers
    pub crccfg_status: bool,
    #[skip(setters)]
    /// Data Ready Status Flag
    pub dr_status: bool,
    #[skip]
    __: B1,
}

/// Analog multiplexer input selection (MUX mode only)
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mux {
    /// MUX_VIN- Input Selection (default = CH1)
    pub vin_n: MuxInput,
    /// MUX_VIN+ Input Selection (default = CH1)
    pub vin_p: MuxInput,
}

/// MUX_VIN Input Selection
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 4]
pub enum MuxInput {
    /// CH0
    Ch0 = 0b0000,
    /// CH1
    Ch1 = 0b0001,
    /// CH2
    #[cfg(feature = "__has_ch2_ch3")]
    Ch2 = 0b0010,
    /// CH3
    #[cfg(feature = "__has_ch2_ch3")]
    Ch3 = 0b0011,
    /// CH4
    #[cfg(feature = "__has_ch4_ch5_ch6_ch7")]
    Ch4 = 0b0100,
    /// CH5
    #[cfg(feature = "__has_ch4_ch5_ch6_ch7")]
    Ch5 = 0b0101,
    /// CH6
    #[cfg(feature = "__has_ch4_ch5_ch6_ch7")]
    Ch6 = 0b0110,
    /// CH7
    #[cfg(feature = "__has_ch4_ch5_ch6_ch7")]
    Ch7 = 0b0111,
    /// AGND
    Agnd = 0b1000,
    /// AVDD
    Avdd = 0b1001,
    /// REFIN–
    Refn = 0b1100,
    /// REFIN+
    Refp = 0b1011,
    /// Internal Temperature Sensor Diode P (Temp Diode P)
    TempP = 0b1101,
    /// Internal Temperature Sensor Diode M (Temp Diode M)
    TempM = 0b1110,
    /// Internal VCM
    Vcm = 0b1111,
}

/// SCAN mode settings
#[cfg(not(feature = "__has_ch2_ch3"))]
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scan {
    #[skip]
    __: B5,
    /// Delay Time (TDLY_SCAN) Between Each Conversion During a SCAN Cycle
    pub dly: Dly,
    /// Differential Channel A (CH0–CH1)
    pub diff_a: bool,
    #[skip]
    __: B3,
    /// Temperature Reading (TEMP)
    pub temp: bool,
    /// Analog Supply Voltage Reading (AVDD)
    pub avdd: bool,
    /// VCM Reading (VCM)
    pub vcm: bool,
    /// Offset Reading (OFFSET)
    pub offset: bool,
    /// Single-Ended Channel CH0
    pub se_ch0: bool,
    /// Single-Ended Channel CH1
    pub se_ch1: bool,
    #[skip]
    __: B6,
}

#[cfg(all(feature = "__has_ch2_ch3", not(feature = "__has_ch4_ch5_ch6_ch7")))]
/// SCAN mode settings
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scan {
    #[skip]
    __: B5,
    /// Delay Time (TDLY_SCAN) Between Each Conversion During a SCAN Cycle
    pub dly: Dly,
    /// Differential Channel A (CH0–CH1)
    pub diff_a: bool,
    /// Differential Channel B (CH2–CH3)
    pub diff_b: bool,
    #[skip]
    __: B2,
    /// Temperature Reading (TEMP)
    pub temp: bool,
    /// Analog Supply Voltage Reading (AVDD)
    pub avdd: bool,
    /// VCM Reading (VCM)
    pub vcm: bool,
    /// Offset Reading (OFFSET)
    pub offset: bool,
    /// Single-Ended Channel CH0
    pub se_ch0: bool,
    /// Single-Ended Channel CH1
    pub se_ch1: bool,
    /// Single-Ended Channel CH2
    pub se_ch2: bool,
    /// Single-Ended Channel CH3
    pub se_ch3: bool,
    #[skip]
    __: B4,
}

#[cfg(feature = "__has_ch4_ch5_ch6_ch7")]
/// SCAN mode settings
#[bitfield]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scan {
    #[skip]
    __: B5,
    /// Delay Time (TDLY_SCAN) Between Each Conversion During a SCAN Cycle
    pub dly: Dly,
    /// Differential Channel A (CH0–CH1)
    pub diff_a: bool,
    /// Differential Channel B (CH2–CH3)
    pub diff_b: bool,
    /// Differential Channel C (CH4–CH5)
    pub diff_c: bool,
    /// Differential Channel D (CH6–CH7)
    pub diff_d: bool,
    /// Temperature Reading (TEMP)
    pub temp: bool,
    /// Analog Supply Voltage Reading (AVDD)
    pub avdd: bool,
    /// VCM Reading (VCM)
    pub vcm: bool,
    /// Offset Reading (OFFSET)
    pub offset: bool,
    /// Single-Ended Channel CH0
    pub se_ch0: bool,
    /// Single-Ended Channel CH1
    pub se_ch1: bool,
    /// Single-Ended Channel CH2
    pub se_ch2: bool,
    /// Single-Ended Channel CH3
    pub se_ch3: bool,
    /// Single-Ended Channel CH4
    pub se_ch4: bool,
    /// Single-Ended Channel CH5
    pub se_ch5: bool,
    /// Single-Ended Channel CH6
    pub se_ch6: bool,
    /// Single-Ended Channel CH7
    pub se_ch7: bool,
}

/// Delay Time Between Each Conversion During SCAN Cycle
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 3]
pub enum Dly {
    /// 512 * DMCLK
    D512 = 0b111,
    /// 256 * DMCLK
    D256 = 0b110,
    /// 128 * DMCLK
    D128 = 0b101,
    /// 64 * DMCLK
    D64 = 0b100,
    /// 32 * DMCLK
    D32 = 0b011,
    /// 16 * DMCLK
    D16 = 0b010,
    /// 8 * DMCLK
    D8 = 0b001,
    /// 0: No delay (default)
    D0 = 0b000,
}

/// Selection Bits for the Time Interval (TTIMER_SCAN)
/// Between Two Consecutive SCAN Cycles CONV_MODE = 1
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer {
    bytes: [u8; 3],
}

impl Timer {
    /// Creates a new `[Timer]`
    #[must_use]
    pub const fn new(val: u32) -> Self {
        let bytes = val.to_be_bytes();
        Self {
            bytes: [bytes[1], bytes[2], bytes[3]],
        }
    }

    /// Returns the value of `timer`.
    #[inline]
    #[must_use]
    pub fn timer(&self) -> u32 {
        u32::from_be_bytes([0, self.bytes[0], self.bytes[1], self.bytes[2]])
    }

    fn from_bytes(bytes: [u8; 3]) -> Self {
        Self { bytes }
    }
}

/// Offset Error Digital Calibration Code
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OffsetCal {
    bytes: [u8; 3],
}

impl OffsetCal {
    /// Creates a new `[OffsetCal]`
    #[must_use]
    pub const fn new(val: i32) -> Self {
        let bytes = val.to_be_bytes();
        Self {
            bytes: [bytes[1], bytes[2], bytes[3]],
        }
    }

    /// Returns the value of `offsetcal`.
    #[inline]
    #[must_use]
    pub fn offsetcal(&self) -> i32 {
        let msb = if self.bytes[0] & 0x80 != 0 { 0xFF } else { 0 };
        i32::from_be_bytes([msb, self.bytes[0], self.bytes[1], self.bytes[2]])
    }

    fn from_bytes(bytes: [u8; 3]) -> Self {
        Self { bytes }
    }
}

/// Gain Error Digital Calibration Code
/// The GAINCAL default value is 0x800000, which provides a gain of 1x.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GainCal {
    bytes: [u8; 3],
}

impl GainCal {
    /// Creates a new `[GainCal]`
    #[must_use]
    pub const fn new(val: u32) -> Self {
        let bytes = val.to_be_bytes();
        Self {
            bytes: [bytes[1], bytes[2], bytes[3]],
        }
    }

    /// Returns the value of `gaincal`.
    #[inline]
    #[must_use]
    pub fn gaincal(&self) -> u32 {
        u32::from_be_bytes([0, self.bytes[0], self.bytes[1], self.bytes[2]])
    }

    fn from_bytes(bytes: [u8; 3]) -> Self {
        Self { bytes }
    }
}

/// Password value for SPI Write mode locking
#[derive(Specifier, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[bits = 8]
pub enum Lock {
    /// Write access is allowed on the full register map. CRC on register map values is not calculated
    Unlocked = 0x00,
    /// Write access is not allowed on the full register map. Only the LOCK register
    /// is writable. CRC on register map is calculated continuously only when DMCLK is running.
    Locked = 0xA5,
}

/// Register
pub trait Register {
    /// Register address
    const ADDRESS: RegisterAddress;
    /// Backing storage for the register
    type Bytes: Default + AsMut<[u8]>;
    /// Register as bytes
    fn bytes(&self) -> &[u8];
    /// Register from bytes
    fn new(bytes: Self::Bytes) -> Self;
}

/// Writeable Register
pub trait WriteableRegister: Register {}

macro_rules! impl_register {
    ($type:ty, $address:expr, $bytes:ty) => {
        impl Register for $type {
            const ADDRESS: RegisterAddress = $address;
            type Bytes = $bytes;

            fn bytes(&self) -> &[u8] {
                &self.bytes
            }

            fn new(bytes: Self::Bytes) -> Self {
                Self::from_bytes(bytes)
            }
        }

        impl WriteableRegister for $type {}
    };
}

impl_register!(Config0, RegisterAddress::CONFIG0, [u8; 1]);
impl_register!(Config1, RegisterAddress::CONFIG1, [u8; 1]);
impl_register!(Config2, RegisterAddress::CONFIG2, [u8; 1]);
impl_register!(Config3, RegisterAddress::CONFIG3, [u8; 1]);
impl_register!(Irq, RegisterAddress::IRQ, [u8; 1]);
impl_register!(Mux, RegisterAddress::MUX, [u8; 1]);
impl_register!(Scan, RegisterAddress::SCAN, [u8; 3]);
impl_register!(Timer, RegisterAddress::TIMER, [u8; 3]);
impl_register!(OffsetCal, RegisterAddress::OFFSETCAL, [u8; 3]);
impl_register!(GainCal, RegisterAddress::GAINCAL, [u8; 3]);

impl Register for Lock {
    const ADDRESS: RegisterAddress = RegisterAddress::LOCK;
    type Bytes = [u8; 1];

    fn bytes(&self) -> &[u8] {
        match self {
            Lock::Unlocked => &[0x00],
            Lock::Locked => &[0xA5],
        }
    }

    fn new(bytes: Self::Bytes) -> Self {
        match bytes[0] {
            0xA5 => Lock::Locked,
            _ => Lock::Unlocked,
        }
    }
}

/// ```should_panic
/// use crate::mcp3x6x::*;
/// fn f(reg: impl WriteableRegister) {}
/// let reg: Mux = unimplemented!();
/// f(reg);
/// ```
/// ```compile_fail
/// use crate::mcp3x6x::*;
/// fn f(reg: impl WriteableRegister){}
/// let reg: CrcCfg = unimplemented!();
/// f(reg);
/// ```
fn _test_read_only() {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bit_order() {
        let mux = Mux::new().with_vin_n(MuxInput::Ch1);
        assert_eq!(mux.bytes, [0b0000_0001]);

        let scan = Scan::new().with_offset(true).with_dly(Dly::D8);
        assert_eq!(scan.bytes, [0b0010_0000, 0b1000_0000, 0b0000_0000]);

        let timer = Timer::new(1);
        assert_eq!(timer.bytes, [0b0000_0000, 0b0000_0000, 0b0000_0001]);
        assert_eq!(timer.timer(), 1);

        let offset_cal = OffsetCal::new(1);
        assert_eq!(offset_cal.bytes, [0b0000_0000, 0b0000_0000, 0b0000_0001]);
        assert_eq!(offset_cal.offsetcal(), 1);
        let offset_cal = OffsetCal::new(-1);
        assert_eq!(offset_cal.bytes, [0b1111_1111, 0b1111_1111, 0b1111_1111]);
        assert_eq!(offset_cal.offsetcal(), -1);
    }
}
