use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_shunt_voltage_raw_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x01]),
        I2cTransaction::read(0b1000000, vec![0b10000011, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let voltage = ina226
        .shunt_voltage_raw()
        .expect("shunt voltage to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(voltage, -32000);
    i2c.done();
}

#[test]
fn read_shunt_voltage_microvolts_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x01]),
        I2cTransaction::read(0b1000000, vec![0b10000011, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let voltage = ina226
        .shunt_voltage_microvolts()
        .expect("shunt voltage to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(voltage, -80000.0);
    i2c.done();
}
