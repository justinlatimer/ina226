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

enum AVG {
    _1 = 0b000,
    _4 = 0b001,
    _16 = 0b010,
    _64 = 0b011,
    _128 = 0b100,
    _256 = 0b101,
    _512 = 0b110,
    _1024 = 0b111,
}

enum VBUSCT {
    _140us = 0b000,
    _204us = 0b001,
    _332us = 0b010,
    _588us = 0b011,
    _1100us = 0b100,
    _2116us = 0b101,
    _4156us = 0b110,
    _8244us = 0b111,
}

const ADDRESS_MIN: u8 = 0b1000000;
const ADDRESS_MAX: u8 = 0b1001111;

const SHUNT_VOLTAGE_LSB_UV: f64 = 2.5; // 2.5 μV.

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
    pub fn shunt_voltage_raw(&mut self) -> Result<i16, E> {
        self.read_i16(Register::ShuntVoltage)
    }

    #[inline(always)]
    pub fn shunt_voltage_microvolts(&mut self) -> Result<f64, E> {
        self.read_i16(Register::ShuntVoltage)
            .map(|raw| (raw as f64) * SHUNT_VOLTAGE_LSB_UV)
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
