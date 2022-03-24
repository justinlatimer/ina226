use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::{Config, AVG, INA226, MODE, VBUSCT, VSHCT};

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
