#![no_std]

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

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum AVG {
    _1 = 0b000,
    _4 = 0b001,
    _16 = 0b010,
    _64 = 0b011,
    _128 = 0b100,
    _256 = 0b101,
    _512 = 0b110,
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

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum VBUSCT {
    _140us = 0b000,
    _204us = 0b001,
    _332us = 0b010,
    _588us = 0b011,
    _1100us = 0b100,
    _2116us = 0b101,
    _4156us = 0b110,
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

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum VSHCT {
    _140us = 0b000,
    _204us = 0b001,
    _332us = 0b010,
    _588us = 0b011,
    _1100us = 0b100,
    _2116us = 0b101,
    _4156us = 0b110,
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

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum MODE {
    PowerDown = 0b000,
    ShuntVoltageTriggered = 0b001,
    BusVoltageTriggered = 0b010,
    ShuntBusVoltageTriggered = 0b011,
    PowerDown2 = 0b100,
    ShuntVoltageContinuous = 0b101,
    BusVoltageContinuous = 0b110,
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

#[derive(Debug, PartialEq)]
pub struct Config {
    pub avg: AVG,
    pub vbusct: VBUSCT,
    pub vshct: VSHCT,
    pub mode: MODE,
}

impl Config {
    pub fn to_value(&self) -> u16 {
        (1 << 14)
            | ((self.avg as u16) << 9)
            | ((self.vbusct as u16) << 6)
            | ((self.vshct as u16) << 3)
            | (self.mode as u16)
    }

    pub fn from_value(value: u16) -> Option<Config> {
        Some(Config {
            avg: AVG::parse(((value >> 9) & 0b111) as u8)?,
            vbusct: VBUSCT::parse(((value >> 6) & 0b111) as u8)?,
            vshct: VSHCT::parse(((value >> 3) & 0b111) as u8)?,
            mode: MODE::parse((value & 0b111) as u8)?,
        })
    }
}

pub const DEFAULT_ADDRESS: u8 = 0b1000000;

const SHUNT_VOLTAGE_LSB_UV: f64 = 2.5; // 2.5 μV.
const BUS_VOLTAGE_LSB_MV: f64 = 1.25; // 1.25 mV.

pub struct INA226<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> INA226<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> INA226<I2C> {
        INA226 { i2c, address }
    }

    #[inline(always)]
    pub fn configuration_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::Configuration)
    }

    #[inline(always)]
    pub fn configuration(&mut self) -> Result<Option<Config>, E> {
        self.read_u16(Register::Configuration)
            .map(|value| Config::from_value(value))
    }

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

    #[inline(always)]
    pub fn shunt_voltage_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::ShuntVoltage)
    }

    #[inline(always)]
    pub fn shunt_voltage_microvolts(&mut self) -> Result<f64, E> {
        self.read_i16(Register::ShuntVoltage)
            .map(|raw| (raw as f64) * SHUNT_VOLTAGE_LSB_UV)
    }

    #[inline(always)]
    pub fn bus_voltage_raw(&mut self) -> Result<u16, E> {
        self.read_u16(Register::BusVoltage)
    }

    #[inline(always)]
    pub fn bus_voltage_millivolts(&mut self) -> Result<f64, E> {
        self.read_u16(Register::BusVoltage)
            .map(|raw| (raw as f64) * BUS_VOLTAGE_LSB_MV)
    }

    #[inline(always)]
    pub fn manufacturer_id(&mut self) -> Result<u16, E> {
        self.read_u16(Register::ManufacturerID)
    }

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

    pub fn destroy(self) -> I2C {
        self.i2c
    }
}
