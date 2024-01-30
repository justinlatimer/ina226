//! This is a platform agnostic Rust driver for the [`INA226`], an I2C output
//! current/voltage/power monitor with alerts, using the [`embedded-hal`] traits.
//!
//! [`INA226`]: https://www.ti.com/product/INA226
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Callibrate the device. See [`callibrate()`].
//! - Read the shunt voltage. See [`shunt_voltage_microvolts()`].
//! - Read the bus voltage. See [`bus_voltage_millivolts()`].
//! - Read the current. See [`current_amps()`].
//! - Read the power. See [`power_watts()`].
//!
//! [`callibrate()`]: struct.INA226.html#method.callibrate
//! [`shunt_voltage_microvolts()`]: struct.INA226.html#method.shunt_voltage_microvolts
//! [`bus_voltage_millivolts()`]: struct.INA226.html#method.bus_voltage_millivolts
//! [`current_amps()`]: struct.INA226.html#method.current_amps
//! [`power_watts()`]: struct.INA226.html#method.power_watts
//!
//! ## The device
//!
//! The INA226 is a current shunt and power monitor with an I2C™- or SMBUS-compatible
//! interface. The device monitors both a shunt voltage drop and bus supply voltage.
//! Programmable calibration value, conversion times, and averaging, combined with
//! an internal multiplier, enable direct readouts of current in amperes and power in
//! watts.
//!
//! The INA226 senses current on common-mode bus voltages that can vary from 0 V to
//! 36 V, independent of the supply voltage. The device operates from a single 2.7-V
//! to 5.5-V supply, drawing a typical of 330 µA of supply current. The device is
//! specified over the operating temperature range between –40°C and 125°C and
//! features up to 16 programmable addresses on the I2C-compatible interface.
//!
//! Datasheet:
//! - [INA226](https://www.ti.com/lit/gpn/ina226)

#![warn(unsafe_code, missing_docs)]
#![no_std]

use bitflags::bitflags;
use byteorder::{BigEndian, ByteOrder};
use embedded_hal::blocking::i2c;

#[repr(u8)]
enum Register {
    Configuration = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Callibration = 0x05,
    MaskEnable = 0x06,
    AlertLimit = 0x07,
    ManufacturerID = 0xFE,
    DieID = 0xFF,
}

/// Determines the number of samples that are collected and averaged.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AVG {
    /// 1 sample averaging
    _1 = 0b000,
    /// 4 sample averaging
    _4 = 0b001,
    /// 16 sample averaging
    _16 = 0b010,
    /// 64 sample averaging
    _64 = 0b011,
    /// 128 sample averaging
    _128 = 0b100,
    /// 256 sample averaging
    _256 = 0b101,
    /// 512 sample averaging
    _512 = 0b110,
    /// 1024 sample averaging
    _1024 = 0b111,
}

impl AVG {
    fn parse(value: u8) -> Option<AVG> {
        match value {
            0b000 => Some(AVG::_1),
            0b001 => Some(AVG::_4),
            0b010 => Some(AVG::_16),
            0b011 => Some(AVG::_64),
            0b100 => Some(AVG::_128),
            0b101 => Some(AVG::_256),
            0b110 => Some(AVG::_512),
            0b111 => Some(AVG::_1024),
            _ => None,
        }
    }
}

/// Sets the conversion time for the bus voltage measurement.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum VBUSCT {
    /// 140us conversion time
    _140us = 0b000,
    /// 204us conversion time
    _204us = 0b001,
    /// 332us conversion time
    _332us = 0b010,
    /// 588us conversion time
    _588us = 0b011,
    /// 1100us conversion time
    _1100us = 0b100,
    /// 2116us conversion time
    _2116us = 0b101,
    /// 4156us conversion time
    _4156us = 0b110,
    /// 8244us conversion time
    _8244us = 0b111,
}

impl VBUSCT {
    fn parse(value: u8) -> Option<VBUSCT> {
        match value {
            0b000 => Some(VBUSCT::_140us),
            0b001 => Some(VBUSCT::_204us),
            0b010 => Some(VBUSCT::_332us),
            0b011 => Some(VBUSCT::_588us),
            0b100 => Some(VBUSCT::_1100us),
            0b101 => Some(VBUSCT::_2116us),
            0b110 => Some(VBUSCT::_4156us),
            0b111 => Some(VBUSCT::_8244us),
            _ => None,
        }
    }
}

/// Sets the conversion time for the shunt voltage measurement.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum VSHCT {
    /// 140us conversion time
    _140us = 0b000,
    /// 204us conversion time
    _204us = 0b001,
    /// 332us conversion time
    _332us = 0b010,
    /// 588us conversion time
    _588us = 0b011,
    /// 1100us conversion time
    _1100us = 0b100,
    /// 2116us conversion time
    _2116us = 0b101,
    /// 4156us conversion time
    _4156us = 0b110,
    /// 8244us conversion time
    _8244us = 0b111,
}

impl VSHCT {
    fn parse(value: u8) -> Option<VSHCT> {
        match value {
            0b000 => Some(VSHCT::_140us),
            0b001 => Some(VSHCT::_204us),
            0b010 => Some(VSHCT::_332us),
            0b011 => Some(VSHCT::_588us),
            0b100 => Some(VSHCT::_1100us),
            0b101 => Some(VSHCT::_2116us),
            0b110 => Some(VSHCT::_4156us),
            0b111 => Some(VSHCT::_8244us),
            _ => None,
        }
    }
}

/// Selects continuous, triggered, or power-down mode of operation.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum MODE {
    /// Power Down (or Shutdown)
    PowerDown = 0b000,
    /// Shunt Voltage, Triggered
    ShuntVoltageTriggered = 0b001,
    /// Bus Voltage, Triggered
    BusVoltageTriggered = 0b010,
    /// Shunt and Bus, Triggered
    ShuntBusVoltageTriggered = 0b011,
    /// Power Down (or Shutdown)
    PowerDown2 = 0b100,
    /// Shunt Voltage, Continuous
    ShuntVoltageContinuous = 0b101,
    /// Bus Voltage, Continuous
    BusVoltageContinuous = 0b110,
    /// Shunt and Bus, Continuous
    ShuntBusVoltageContinuous = 0b111,
}

impl MODE {
    fn parse(value: u8) -> Option<MODE> {
        match value {
            0b000 => Some(MODE::PowerDown),
            0b001 => Some(MODE::ShuntVoltageTriggered),
            0b010 => Some(MODE::BusVoltageTriggered),
            0b011 => Some(MODE::ShuntBusVoltageTriggered),
            0b100 => Some(MODE::PowerDown2),
            0b101 => Some(MODE::ShuntVoltageContinuous),
            0b110 => Some(MODE::BusVoltageContinuous),
            0b111 => Some(MODE::ShuntBusVoltageContinuous),
            _ => None,
        }
    }
}

/// The state of the configuration register.
#[derive(Debug, PartialEq)]
pub struct Config {
    /// Averaging Mode
    pub avg: AVG,
    /// Bus Voltage Conversion Time
    pub vbusct: VBUSCT,
    /// Shunt Voltage Conversion Time
    pub vshct: VSHCT,
    /// Operating Mode
    pub mode: MODE,
}

impl Config {
    fn to_value(&self) -> u16 {
        (1 << 14)
            | ((self.avg as u16) << 9)
            | ((self.vbusct as u16) << 6)
            | ((self.vshct as u16) << 3)
            | (self.mode as u16)
    }

    fn from_value(value: u16) -> Option<Config> {
        Some(Config {
            avg: AVG::parse(((value >> 9) & 0b111) as u8)?,
            vbusct: VBUSCT::parse(((value >> 6) & 0b111) as u8)?,
            vshct: VSHCT::parse(((value >> 3) & 0b111) as u8)?,
            mode: MODE::parse((value & 0b111) as u8)?,
        })
    }
}

bitflags! {
    /// The Mask/Enable Register selects the function that is enabled to control the
    /// Alert pin as well as how that pin functions. If multiple functions are
    /// enabled, the highest significant bit position Alert Function (D15-D11) takes
    /// priority and responds to the Alert Limit Register.
    pub struct MaskEnableFlags: u16 {
        /// Shunt Voltage Over-Voltage
        /// Bit 15
        /// Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement
        /// following a conversion exceeds the value programmed in the Alert Limit Register.
        const SOL = 1 << 15;

        /// Shunt Voltage Under-Voltage
        /// Bit 14
        /// Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement
        /// following a conversion drops below the value programmed in the Alert Limit Register.
        const SUL = 1 << 14;

        /// Bus Voltage Over-Voltage
        /// Bit 13
        /// Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement
        /// following a conversion exceeds the value programmed in the Alert Limit Register.
        const BOL = 1 << 13;

        /// Bus Voltage Under-Voltage
        /// Bit 12
        /// Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement
        /// following a conversion drops below the value programmed in the Alert Limit Register.
        const BUL = 1 << 12;

        /// Power Over-Limit
        /// Bit 11
        /// Setting this bit high configures the Alert pin to be asserted if the Power calculation made
        /// following a bus voltage measurement exceeds the value programmed in the Alert Limit Register.
        const POL = 1 << 11;

        /// Conversion Ready
        /// Bit 10
        /// Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag, Bit 3, is
        /// asserted indicating that the device is ready for the next conversion.
        const CNVR = 1 << 10;

        /// Alert Function Flag
        /// Bit 4
        /// While only one Alert Function can be monitored at the Alert pin at a time, the Conversion Ready can also
        /// be enabled to assert the Alert pin. Reading the Alert Function Flag following an alert allows the user
        /// to determine if the Alert Function was the source of the Alert.
        ///
        /// When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit clears only when the
        /// Mask/Enable Register is read. When the Alert Latch Enable bit is set to Transparent mode, the Alert
        /// Function Flag bit is cleared following the next conversion that does not result in an Alert condition.
        const AFF = 1 << 4;

        /// Conversion Ready Flag
        /// Bit 3
        /// Although the device can be read at any time, and the data from the last conversion is available, the
        /// Conversion Ready Flag bit is provided to help coordinate one-shot or triggered conversions. The
        /// Conversion Ready Flag bit is set after all conversions, averaging, and multiplications are complete.
        /// Conversion Ready Flag bit clears under the following conditions:
        /// 1.) Writing to the Configuration Register (except for Power-Down selection)
        /// 2.) Reading the Mask/Enable Register
        const CVRF = 1 << 3;

        /// Math Overflow Flag
        /// Bit 2
        /// This bit is set to '1' if an arithmetic operation resulted in an overflow error. It indicates that
        /// current and power data may be invalid.
        const OVF = 1 << 2;

        /// Alert Polarity bit; sets the Alert pin polarity.
        /// Bit 1
        /// 1 = Inverted (active-high open collector)
        /// 0 = Normal (active-low open collector) (default)
        const APOL = 1 << 1;

        /// Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits.
        /// Bit 0
        /// 1 = Latch enabled
        /// 0 = Transparent (default)
        /// When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit
        /// resets to the idle states when the fault has been cleared. When the Alert Latch Enable
        /// bit is set to Latch mode, the Alert pin and Alert Flag bit remains active following a
        /// fault until the Mask/Enable Register has been read.
        const LEN = 1 << 0;
    }
}

/// Default address of INA226 devices.
pub const DEFAULT_ADDRESS: u8 = 0b1000000;

const SHUNT_VOLTAGE_LSB_UV: f64 = 2.5; // 2.5 μV.
const BUS_VOLTAGE_LSB_MV: f64 = 1.25; // 1.25 mV.
const SCALING_VALUE: f64 = 0.00512; // An internal fixed value used to ensure scaling is maintained properly.
const POWER_LSB_FACTOR: f64 = 25.0; // The Power Register LSB is internally programmed to equal 25 times the programmed value of the Current_LSB.

#[inline(always)]
fn calculate_calibration_value(shunt_resistance: f64, current_lsb: f64) -> u16 {
    (SCALING_VALUE / (current_lsb * shunt_resistance)) as u16
}

#[inline(always)]
fn calculate_current_lsb(current_expected_max: f64) -> f64 {
    current_expected_max / ((1 << 15) as f64)
}

struct Callibration {
    current_lsb: f64,
    power_lsb: f64,
}

/// INA226 voltage/current/power monitor
pub struct INA226<I2C> {
    i2c: I2C,
    address: u8,
    callibration: Option<Callibration>,
}

impl<I2C, E> INA226<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    /// Create a new instance of an INA226 device.
    pub fn new(i2c: I2C, address: u8) -> INA226<I2C> {
        INA226 {
            i2c,
            address,
            callibration: None,
        }
    }

    /// Gets the raw configuration value.
    #[inline(always)]
    pub fn configuration_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::Configuration)
    }

    /// Gets the configuration.
    #[inline(always)]
    pub fn configuration(&mut self) -> Result<Option<Config>, E> {
        self.read_u16(Register::Configuration)
            .map(Config::from_value)
    }

    /// Set the configuration of the device.
    #[inline(always)]
    pub fn set_configuration(&mut self, config: &Config) -> Result<(), E> {
        let value = config.to_value();
        self.i2c.write(
            self.address,
            &[
                Register::Configuration as u8,
                (value >> 8) as u8,
                value as u8,
            ],
        )
    }

    /// Gets the raw shunt voltage measurement.
    #[inline(always)]
    pub fn shunt_voltage_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::ShuntVoltage)
    }

    /// Gets the shunt voltage in microvolts.
    #[inline(always)]
    pub fn shunt_voltage_microvolts(&mut self) -> Result<f64, E> {
        self.read_i16(Register::ShuntVoltage)
            .map(|raw| (raw as f64) * SHUNT_VOLTAGE_LSB_UV)
    }

    /// Gets the raw bus voltage measurement.
    #[inline(always)]
    pub fn bus_voltage_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::BusVoltage)
    }

    /// Gets the bus voltage in millivolts.
    #[inline(always)]
    pub fn bus_voltage_millivolts(&mut self) -> Result<f64, E> {
        self.read_u16(Register::BusVoltage)
            .map(|raw| (raw as f64) * BUS_VOLTAGE_LSB_MV)
    }

    /// Gets the raw calculated power being delivered to the load.
    /// Returns zero if callibration has not been performed.
    #[inline(always)]
    pub fn power_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::Power)
    }

    /// Gets the calculated power (in Watts) being delivered to the load.
    /// Requires callibration.
    #[inline(always)]
    pub fn power_watts(&mut self) -> Result<Option<f64>, E> {
        if let Some(Callibration { power_lsb, .. }) = self.callibration {
            self.read_u16(Register::Power)
                .map(|raw| Some((raw as f64) * power_lsb))
        } else {
            Ok(None)
        }
    }

    /// Gets the calculated current flowing through the shunt resistor.
    /// Returns zero if callibration has not been performed.
    #[inline(always)]
    pub fn current_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::Current)
    }

    /// Gets the calculated current (in Amps) flowing through the shunt resistor.
    /// Requires callibration.
    #[inline(always)]
    pub fn current_amps(&mut self) -> Result<Option<f64>, E> {
        if let Some(Callibration { current_lsb, .. }) = self.callibration {
            self.read_i16(Register::Current)
                .map(|raw| Some((raw as f64) * current_lsb))
        } else {
            Ok(None)
        }
    }

    /// Gets the callibration register, which controls full-scale
    /// range of current and power measurements.
    #[inline(always)]
    pub fn callibration(&mut self) -> Result<u16, E> {
        self.read_u16(Register::Callibration)
    }

    /// Set the callibration register directly.
    /// NB: after calling this, only `_raw` methods can be used.
    #[inline(always)]
    pub fn set_callibration_raw(&mut self, value: u16) -> Result<(), E> {
        self.callibration = None;
        self.i2c.write(
            self.address,
            &[
                Register::Callibration as u8,
                (value >> 8) as u8,
                value as u8,
            ],
        )
    }

    /// Calibrate the sensitvity of the current and power values.
    #[inline(always)]
    pub fn callibrate(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), E> {
        let current_lsb = calculate_current_lsb(current_expected_max);
        let power_lsb = current_lsb * POWER_LSB_FACTOR;
        self.callibration = Some(Callibration {
            current_lsb,
            power_lsb,
        });
        let value = calculate_calibration_value(shunt_resistance, current_lsb);
        self.i2c.write(
            self.address,
            &[
                Register::Callibration as u8,
                (value >> 8) as u8,
                value as u8,
            ],
        )
    }

    /// Get the Alert configuration and Conversion Ready flag.
    #[inline(always)]
    pub fn mask_enable(&mut self) -> Result<MaskEnableFlags, E> {
        self.read_u16(Register::MaskEnable)
            .map(MaskEnableFlags::from_bits_truncate)
    }

    /// Set the Alert configuration and Conversion Ready flags.
    #[inline(always)]
    pub fn set_mask_enable(&mut self, flags: MaskEnableFlags) -> Result<(), E> {
        let value = flags.bits();
        self.i2c.write(
            self.address,
            &[Register::MaskEnable as u8, (value >> 8) as u8, value as u8],
        )
    }

    /// Get the limit value to compare to the selected Alert function.
    #[inline(always)]
    pub fn alert_limit(&mut self) -> Result<u16, E> {
        self.read_u16(Register::AlertLimit)
    }


    /// Set the Alert Limit register.
    #[inline(always)]
    pub fn set_alert_limit(&mut self, value: u16) -> Result<(), E> {

        self.i2c.write(
            self.address,
            &[
                Register::AlertLimit as u8,
                (value >> 8) as u8,
                value as u8,
            ],
        )
    }

    /// Get the unique manufacturer identification number
    #[inline(always)]
    pub fn manufacturer_id(&mut self) -> Result<u16, E> {
        self.read_u16(Register::ManufacturerID)
    }

    /// Get the unique die identification number.
    #[inline(always)]
    pub fn die_id(&mut self) -> Result<u16, E> {
        self.read_u16(Register::DieID)
    }

    fn read_i16(&mut self, register: Register) -> Result<i16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address, &[register as u8])?;
        self.i2c.read(self.address, &mut buf)?;
        Ok(BigEndian::read_i16(&buf))
    }

    fn read_u16(&mut self, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address, &[register as u8])?;
        self.i2c.read(self.address, &mut buf)?;
        Ok(BigEndian::read_u16(&buf))
    }

    /// Destroy the INA226 instance and return the I2C.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::{
        calculate_calibration_value, calculate_current_lsb, Config, AVG, MODE, VBUSCT, VSHCT,
    };

    #[test]
    fn calculate_current_lsb_works() {
        let result = calculate_current_lsb(15.0);
        assert_eq!(result, 0.000457763671875);
    }

    #[test]
    fn calculate_calibration_value_works() {
        let shunt_resistance: f64 = 0.002; // 2mOhm
        let current_lsb: f64 = 0.001; // 1 mA
        let result = calculate_calibration_value(shunt_resistance, current_lsb);
        assert_eq!(result, 2560);
    }

    #[test]
    fn configuration_to_value_works() {
        let config = Config {
            avg: AVG::_1,
            vbusct: VBUSCT::_1100us,
            vshct: VSHCT::_1100us,
            mode: MODE::ShuntBusVoltageContinuous,
        };

        assert_eq!(config.to_value(), 0b0100000100100111);
    }

    #[test]
    fn configuration_from_value_works() {
        let config = Config::from_value(0b0100000100100111).expect("config to be parsed");

        assert_eq!(
            config,
            Config {
                avg: AVG::_1,
                vbusct: VBUSCT::_1100us,
                vshct: VSHCT::_1100us,
                mode: MODE::ShuntBusVoltageContinuous,
            }
        );
    }
}
