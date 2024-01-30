use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_manufacturer_id_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0xFE]),
        I2cTransaction::read(0b1000000, vec![0b01010100, 0b01001001]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let id = ina226.manufacturer_id().expect("id to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(id, 0b0101010001001001);
    i2c.done();
}

#[test]
fn read_die_id_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0xFF]),
        I2cTransaction::read(0b1000000, vec![0b00100010, 0b01100000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let id = ina226.die_id().expect("id to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(id, 0b0010001001100000);
    i2c.done();
}
