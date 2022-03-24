use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_bus_voltage_raw_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x02]),
        I2cTransaction::read(0b1000000, vec![0b00000011, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let voltage = ina226
        .bus_voltage_raw()
        .expect("bus voltage to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(voltage, 768);
    i2c.done();
}

#[test]
fn read_bus_voltage_microvolts_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x02]),
        I2cTransaction::read(0b1000000, vec![0b00000011, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let voltage = ina226
        .bus_voltage_millivolts()
        .expect("bus voltage to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(voltage, 960.0);
    i2c.done();
}


#[test]
fn read_bus_voltage_microvolts_full_range_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x02]),
        I2cTransaction::read(0b1000000, vec![0x7F, 0xFF]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let voltage = ina226
        .bus_voltage_millivolts()
        .expect("bus voltage to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(voltage, 40958.75);
    i2c.done();
}

