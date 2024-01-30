use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_alert_limit_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x07]),
        I2cTransaction::read(0b1000000, vec![0b00000100, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let limit = ina226.alert_limit().expect("alert limit returned");

    let mut i2c = ina226.destroy();

    assert_eq!(limit, 0x0400);
    i2c.done();
}

#[test]
fn set_alert_limit_works() {
    let expectations = [I2cTransaction::write(
        0b1000000,
        vec![0x07, 0b00000100, 0b00000000],
    )];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226.set_alert_limit(0x0400).expect("alert limit written");

    let mut i2c = ina226.destroy();

    i2c.done();
}
