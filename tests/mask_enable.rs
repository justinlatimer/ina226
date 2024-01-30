use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use ina226::{MaskEnableFlags, INA226};

#[test]
fn read_mask_enable_works() {
    let expectations = [
        I2cTransaction::write(0b1000000, vec![0x06]),
        I2cTransaction::read(0b1000000, vec![0b00000100, 0b00000000]),
    ];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    let flags = ina226.mask_enable().expect("mask enable flags returned");

    let mut i2c = ina226.destroy();

    assert_eq!(flags, MaskEnableFlags::CNVR);
    i2c.done();
}

#[test]
fn write_mask_enable_works() {
    let expectations = [I2cTransaction::write(
        0b1000000,
        vec![0x06, 0b00000100, 0b00000000],
    )];
    let i2c = I2cMock::new(&expectations);

    let mut ina226 = INA226::new(i2c, 0b1000000);
    ina226
        .set_mask_enable(MaskEnableFlags::CNVR)
        .expect("mask enable flags set");

    let mut i2c = ina226.destroy();

    i2c.done();
}
