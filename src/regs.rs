use crate::RegisterAddress;
use bitfield_struct::{bitenum, bitfield};

/// ADC Operating mode, Master Clock mode and Input Bias Current Source mode
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
pub struct Config0 {
    /// ADC Operating Mode Selection
    #[bits(2)]
    pub adc_mode: AdcMode,
    /// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-)
    #[bits(2)]
    pub cs_sel: CsSel,
    /// Clock Selection
    #[bits(2)]
    pub clk_sel: ClkSel,
    #[bits(2)]
    __: u8,
}

/// Clock Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClkSel {
    /// Internal clock is selected and AMCLK is present on the analog master clock output pin
    InternalClockAndAMCLK = 0b11,
    /// Internal clock is selected and no clock output is present on the CLK pin
    InternalClock = 0b10,
    /// External digital clock (default)
    #[fallback]
    ExternalDigitalClock = 0b00,
}

/// Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CsSel {
    /// 15 µA is applied to the ADC inputs
    Cs15uA = 0b11,
    /// 3.7 µA is applied to the ADC inputs
    Cs3_7uA = 0b10,
    /// 0.9 µA is applied to the ADC inputs
    Cs0_9uA = 0b01,
    /// No current source is applied to the ADC inputs (default)
    #[fallback]
    CsOff = 0b00,
}

/// ADC Operating Mode Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcMode {
    /// 11 = ADC Conversion mode
    Conversion = 0b11,
    /// 10 = ADC Standby mode
    Standby = 0b10,
    /// 00 = ADC Shutdown mode (default)
    #[fallback]
    ShutdownDefault = 0b00,
}

/// Prescale and OSR settings
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
pub struct Config1 {
    #[bits(2)]
    __: u8,
    /// Oversampling Ratio for Delta-Sigma A/D Conversion
    #[bits(4)]
    pub osr: Osr,
    /// Prescaler Value Selection for AMCLK
    #[bits(2)]
    pub pre: Pre,
}

/// Prescaler Value Selection for AMCLK
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pre {
    /// AMCLK = MCLK/8
    MclkDiv8 = 0b11,
    /// AMCLK = MCLK/4
    MclkDiv4 = 0b10,
    /// AMCLK = MCLK/2
    MclkDiv2 = 0b01,
    /// AMCLK = MCLK (default)
    #[fallback]
    Mclk = 0b00,
}

/// Oversampling Ratio for Delta-Sigma A/D Conversion
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    #[fallback]
    Osr32 = 0b0000,
}

///  ADC boost and gain settings, auto-zeroing settings for analog
/// multiplexer, voltage reference and ADC
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
pub struct Config2 {
    #[bits(2)]
    __: u8,
    /// Auto-Zeroing MUX Setting
    pub az_mux: bool,
    /// ADC Gain Selection
    #[bits(3)]
    pub gain: Gain,
    /// ADC Bias Current Selection
    #[bits(2)]
    pub boost: Boost,
}

/// ADC Bias Current Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Boost {
    /// ADC channel has current ×2
    X2 = 0b11,
    /// ADC channel has current ×1 (default)
    X1 = 0b10,
    /// ADC channel has current ×0.66
    X0_66 = 0b01,
    /// ADC channel has current ×0.5
    #[fallback]
    X0_5 = 0b00,
}

/// ADC Gain Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    #[fallback]
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
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
pub struct Config3 {
    /// Enable Digital Gain Calibration (default = false)
    pub en_gaincal: bool,
    /// Enable Digital Offset Calibration (default = false)
    pub en_offcal: bool,
    /// CRC Checksum Selection on Read Communications (default = false)
    pub en_crccom: bool,
    /// CRC Checksum Format Selection on Read Communications
    #[bits(1)]
    pub crc_format: CrcFormat,
    /// ADC Output Data Format Selection
    #[bits(2)]
    pub data_format: DataFormat,
    /// Conversion Mode Selection
    #[bits(2)]
    pub conv_mode: ConvMode,
}

/// CRC Checksum Format Selection on Read Communications
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrcFormat {
    /// 32-bit wide (CRC-16 followed by 16 zeros)
    Wide32 = 1,
    /// 16-bit wide (CRC-16 only) (default)
    #[fallback]
    Wide16 = 0,
}

/// Conversion Mode Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConvMode {
    /// Continuous Conversion mode or continuous conversion cycle in SCAN mode
    Continuous = 0b11,
    /// One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE[1:0] to ‘10’ (standby) at
    /// the end of the conversion or at the end of the conversion cycle in SCAN mode.
    OneShotStandby = 0b10,
    /// One-shot conversion or one-shot cycle in SCAN mode. It sets ADC_MODE[1:0] to ‘0x’ (ADC
    /// Shutdown) at the end of the conversion or at the end of the conversion cycle in SCAN mode (default).
    #[fallback]
    OneShotShutdown = 0b00,
}

/// ADC Output Data Format Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataFormat {
    /// 32-bit (25-bit right justified data + Channel ID): CHID[3:0] + SGN extension (4 bits) + 24-bit ADC
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
    #[fallback]
    Format24Default = 0b00,
}

/// IRQ Status bits and IRQ mode settings; enable for Fast commands and
/// for conversion start pulse
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
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
    #[bits(access = RO)]
    /// Power-On Reset Status Flag
    pub por_status: bool,
    #[bits(access = RO)]
    /// CRC Error Status Flag for Configuration Registers
    pub crccfg_status: bool,
    #[bits(access = RO)]
    /// Data Ready Status Flag
    pub dr_status: bool,
    #[bits(1)]
    __: u8,
}

/// Analog multiplexer input selection (MUX mode only)
#[cfg_attr(not(feature = "defmt"), bitfield(u8))]
#[cfg_attr(feature = "defmt", bitfield(u8, defmt=true))]
pub struct Mux {
    /// MUX_VIN- Input Selection (default = CH1)
    #[bits(4)]
    pub vin_n: MuxInput,
    /// MUX_VIN+ Input Selection (default = CH1)
    #[bits(4)]
    pub vin_p: MuxInput,
}

/// MUX_VIN Input Selection
#[bitenum]
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MuxInput {
    /// CH0
    #[fallback]
    Ch0 = 0b0000,
    /// CH1
    Ch1 = 0b0001,
    /// CH2
    #[cfg(any(feature = "mcp3464", feature = "mcp3564", feature = "mcp3462", feature = "mcp3562"))]
    Ch2 = 0b0010,
    /// CH3
    #[cfg(any(feature = "mcp3464", feature = "mcp3564", feature = "mcp3462", feature = "mcp3562"))]
    Ch3 = 0b0011,
    /// CH4
    #[cfg(any(feature = "mcp3464", feature = "mcp3564"))]
    Ch4 = 0b0100,
    /// CH5
    #[cfg(any(feature = "mcp3464", feature = "mcp3564"))]
    Ch5 = 0b0101,
    /// CH6
    #[cfg(any(feature = "mcp3464", feature = "mcp3564"))]
    Ch6 = 0b0110,
    /// CH7
    #[cfg(any(feature = "mcp3464", feature = "mcp3564"))]
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

/// Internal Registers with 8 bits of data
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum Register8Bit {
    /// [`Config0`]
    Config0(Config0) = RegisterAddress::CONFIG0 as u8,
    /// [`Config1`]
    Config1(Config1) = RegisterAddress::CONFIG1 as u8,
    /// [`Config2`]
    Config2(Config2) = RegisterAddress::CONFIG2 as u8,
    /// [`Config3`]
    Config3(Config3) = RegisterAddress::CONFIG3 as u8,
    /// [`Irq`]
    Irq(Irq) = RegisterAddress::IRQ as u8,
    /// [`Mux`]
    Mux(Mux) = RegisterAddress::MUX as u8,
}

impl Register8Bit {
    pub(super) fn addr_value(self) -> (RegisterAddress, u8) {
        match self {
            Register8Bit::Config0(data) => (RegisterAddress::CONFIG0, data.into_bits()),
            Register8Bit::Config1(data) => (RegisterAddress::CONFIG1, data.into_bits()),
            Register8Bit::Config2(data) => (RegisterAddress::CONFIG2, data.into_bits()),
            Register8Bit::Config3(data) => (RegisterAddress::CONFIG3, data.into_bits()),
            Register8Bit::Irq(data) => (RegisterAddress::IRQ, data.into_bits()),
            Register8Bit::Mux(data) => (RegisterAddress::MUX, data.into_bits()),
        }
    }
}
macro_rules! impl_register {
    (
        $(
            $ty:ty => $variant:ident
        ),* $(,)?
    ) => {
        $(
            impl From<$ty> for Register8Bit {
                #[inline]
                fn from(value: $ty) -> Self {
                    Register8Bit::$variant(value)
                }
            }
        )*
    };
}

impl_register!(
    Config0 => Config0,
    Config1 => Config1,
    Config2 => Config2,
    Config3 => Config3,
    Irq     => Irq,
    Mux     => Mux,
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bit_order() {
        let mux = Mux::new().with_vin_n(MuxInput::Ch1);
        assert_eq!(mux.into_bits(), 0b0000_0001);
    }
}
