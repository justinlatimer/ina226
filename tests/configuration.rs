use embedded_hal_mock::eh0::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::INA226;

#[test]
fn read_configuration_raw_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x00]),
        I2cTransaction::read(0b1000000, vec![0b01000001, 0b00100111]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let config = ina226
        .configuration_raw()
        .expect("configuration to be returned");

    let mut i2c = ina226.destroy();

    assert_eq!(config, 0b0100000100100111);
    i2c.done();
}
