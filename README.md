# mcp3x6x

[<img alt="github" src="https://img.shields.io/badge/github-grey?style=for-the-badge&labelColor=555555&logo=github">](https://github.com/WMT-GmbH/mcp3x6x)
[<img alt="Crates.io" src="https://img.shields.io/crates/v/mcp3x6x?logo=rust&style=for-the-badge">](https://crates.io/crates/mcp3x6x)
[<img alt="docs.rs" src="https://img.shields.io/badge/docs.rs-mcp3x6x-blue?style=for-the-badge&logoColor=white&logo=data:image/svg+xml;base64,PHN2ZyByb2xlPSJpbWciIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyIgdmlld0JveD0iMCAwIDUxMiA1MTIiPjxwYXRoIGZpbGw9IiNmNWY1ZjUiIGQ9Ik00ODguNiAyNTAuMkwzOTIgMjE0VjEwNS41YzAtMTUtOS4zLTI4LjQtMjMuNC0zMy43bC0xMDAtMzcuNWMtOC4xLTMuMS0xNy4xLTMuMS0yNS4zIDBsLTEwMCAzNy41Yy0xNC4xIDUuMy0yMy40IDE4LjctMjMuNCAzMy43VjIxNGwtOTYuNiAzNi4yQzkuMyAyNTUuNSAwIDI2OC45IDAgMjgzLjlWMzk0YzAgMTMuNiA3LjcgMjYuMSAxOS45IDMyLjJsMTAwIDUwYzEwLjEgNS4xIDIyLjEgNS4xIDMyLjIgMGwxMDMuOS01MiAxMDMuOSA1MmMxMC4xIDUuMSAyMi4xIDUuMSAzMi4yIDBsMTAwLTUwYzEyLjItNi4xIDE5LjktMTguNiAxOS45LTMyLjJWMjgzLjljMC0xNS05LjMtMjguNC0yMy40LTMzLjd6TTM1OCAyMTQuOGwtODUgMzEuOXYtNjguMmw4NS0zN3Y3My4zek0xNTQgMTA0LjFsMTAyLTM4LjIgMTAyIDM4LjJ2LjZsLTEwMiA0MS40LTEwMi00MS40di0uNnptODQgMjkxLjFsLTg1IDQyLjV2LTc5LjFsODUtMzguOHY3NS40em0wLTExMmwtMTAyIDQxLjQtMTAyLTQxLjR2LS42bDEwMi0zOC4yIDEwMiAzOC4ydi42em0yNDAgMTEybC04NSA0Mi41di03OS4xbDg1LTM4Ljh2NzUuNHptMC0xMTJsLTEwMiA0MS40LTEwMi00MS40di0uNmwxMDItMzguMiAxMDIgMzguMnYuNnoiPjwvcGF0aD48L3N2Zz4K">](https://docs.rs/mcp3x6x)

`no_std` library for the MCP3x6x(R) family of analog digital converters.

Supports:

* MCP3461
* MCP3462
* MCP3464
* MCP3561
* MCP3562
* MCP3564
* MCP3461R
* MCP3462R
* MCP3464R
* MCP3465R
* MCP3561R
* MCP3562R
* MCP3564R
* MCP3565R

## Features:

* Activate one of the device features to enable support for the corresponding device.
* `defmt` feature implements `defmt::Format` for all registers.

## Basic usage:

```rust
use mcp3x6x::{FastCommand, MCP3x6x, ToVoltageConverter24bit, Irq, ClkSel, Config0};

// use 3.3V as Vref+ and 0V as Vref-
const TO_VOLT: ToVoltageConverter24bit = ToVoltageConverter24bit::new(3.3, 0.0, mcp3x6x::Gain::X1);

// spi is a struct implementing embedded_hal::spi::SpiDevice.
// irq is an input pin attached to the IRQ pin of the ADC.

let mut adc = MCP3x6x::new(spi);

// use internal clock
let config0 = Config0::new().with_clk_sel(ClkSel::InternalClock);
adc.write_register(config0).unwrap();

// disable en_stp
let irq = Irq::new().with_en_fastcmd(true);
adc.write_register(irq).unwrap();

// be ready for conversions
adc.fast_command(FastCommand::Standby).unwrap();

loop {
    adc.fast_command(FastCommand::ConversionStart);
    while irq_pin.is_high() {}
    let sample = adc.read_24_bit_adc_data().unwrap();
    let voltage = TO_VOLT.to_volt(sample);
}
```

#### License

<sup>
Licensed under either of <a href="LICENSE-APACHE">Apache License, Version
2.0</a> or <a href="LICENSE-MIT">MIT license</a> at your option.
</sup>

<br>

<sub>
Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this crate by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
</sub>