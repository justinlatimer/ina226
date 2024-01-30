use embedded_hal_mock::eh0::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_callibration_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x05]),
        I2cTransaction::read(0b1000000, vec![0x01, 0x02]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let callibration = ina226.callibration().expect("callibration to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(callibration, 0x102);
    i2c.done();
}

#[test]
fn write_callibration_raw_works() {
    let expectations = [I2cTransaction::write(0b1000000, vec![0x05, 0x01, 0x02])];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226
        .set_callibration_raw(0x102)
        .expect("write to be successfull");

    let mut i2c = ina226.destroy();

    i2c.done();
}

#[test]
fn callibrate_works() {
    let expectations = [I2cTransaction::write(0b1000000, vec![0x05, 0x0A, 0x00])];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226
        .callibrate(0.002, 32.768)
        .expect("callibration works");

    let mut i2c = ina226.destroy();
    i2c.done();
}
