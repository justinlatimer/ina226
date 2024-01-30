use approx::assert_relative_eq;
use embedded_hal_mock::eh0::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_current_without_callibration_returns_none() {
    let i2c = I2cMock::new(&[]);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let current = ina226.current_amps().expect("current to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(current, None);
    i2c.done();
}

#[test]
fn read_current_raw_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x04]),
        I2cTransaction::read(0b1000000, vec![0x01, 0x04]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let current = ina226.current_raw().expect("current to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(current, 0x104);
    i2c.done();
}

#[test]
fn read_current_with_callibration_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x05, 0x0A, 0x00]),
        I2cTransaction::write(0b1000000, vec![0x04]),
        I2cTransaction::read(0b1000000, vec![0x27, 0x10]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226
        .callibrate(0.002, 32.768)
        .expect("callibration works");
    let current = ina226.current_amps().expect("current to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(current, Some(10.0));
    i2c.done();
}

#[test]
fn read_power_without_callibration_returns_none() {
    let i2c = I2cMock::new(&[]);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let power = ina226.power_watts().expect("power to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(power, None);
    i2c.done();
}

#[test]
fn read_power_raw_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x03]),
        I2cTransaction::read(0b1000000, vec![0x01, 0x03]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let power = ina226.power_raw().expect("power to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(power, 0x103);
    i2c.done();
}

#[test]
fn read_power_with_callibration_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x05, 0x0A, 0x00]),
        I2cTransaction::write(0b1000000, vec![0x03]),
        I2cTransaction::read(0b1000000, vec![0x12, 0xB8]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226
        .callibrate(0.002, 32.768)
        .expect("callibration works");
    let power = ina226.power_watts().expect("power to be returned").unwrap();

    let mut i2c = ina226.destroy();

    assert_relative_eq!(power, 119.8);
    i2c.done();
}
