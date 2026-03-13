#include "controller/icm20948_controller.hpp"
#include "main.h"

bolt::controller::ICM20948Controller::ICM20948Controller(bolt::spi::SpiSyncPort *spi)
    : spi_(spi), currentBank_(0xFF)
{
}

bool bolt::controller::ICM20948Controller::init(GyroFS gyroFs, AccelFS accelFs)
{
    reset();

    if (!isConnected())
    {
        return false;
    }

    // Wake up: clear sleep bit, auto-select best clock source
    selectBank(USER_BANK_0);
    spi_->writeRegister(UB0_PWR_MGMT_1, 0x01);
    HAL_Delay(30);

    // Enable all accel and gyro axes
    spi_->writeRegister(UB0_PWR_MGMT_2, 0x00);

    // Enable ODR start-time alignment
    selectBank(USER_BANK_2);
    spi_->writeRegister(UB2_ODR_ALIGN_EN, 0x01);

    configureGyro(gyroFs);
    configureAccel(accelFs);

    // Configure magnetometer via I2C master passthrough
    i2cMasterEnable();
    configureMag(MagMode::CONTINUOUS_100HZ);

    // Set up SLV0 to continuously read 8 bytes from AK09916 (ST1 through ST2)
    selectBank(USER_BANK_3);
    spi_->writeRegister(UB3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
    spi_->writeRegister(UB3_I2C_SLV0_REG, AK09916_ST1);
    spi_->writeRegister(UB3_I2C_SLV0_CTRL, 0x88);

    selectBank(USER_BANK_0);

    return true;
}

bool bolt::controller::ICM20948Controller::isConnected()
{
    selectBank(USER_BANK_0);
    uint8_t whoAmI = spi_->readRegister(UB0_WHO_AM_I);
    return whoAmI == ICM20948_WHO_AM_I_VALUE;
}

void bolt::controller::ICM20948Controller::readAccel(int16_t &x, int16_t &y, int16_t &z)
{
    selectBank(USER_BANK_0);
    uint8_t buf[6];
    spi_->readRegisters(UB0_ACCEL_XOUT_H, buf, 6);

    x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    y = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    z = static_cast<int16_t>((buf[4] << 8) | buf[5]);
}

void bolt::controller::ICM20948Controller::readGyro(int16_t &x, int16_t &y, int16_t &z)
{
    selectBank(USER_BANK_0);
    uint8_t buf[6];
    spi_->readRegisters(UB0_GYRO_XOUT_H, buf, 6);

    x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    y = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    z = static_cast<int16_t>((buf[4] << 8) | buf[5]);
}

void bolt::controller::ICM20948Controller::readMag(int16_t &x, int16_t &y, int16_t &z)
{
    selectBank(USER_BANK_0);
    uint8_t buf[8];
    spi_->readRegisters(UB0_EXT_SLV_SENS_DATA_00, buf, 8);

    // buf[0] = ST1, buf[1..6] = XL,XH,YL,YH,ZL,ZH, buf[7] = ST2
    if (buf[0] & 0x01)
    {
        x = static_cast<int16_t>((buf[2] << 8) | buf[1]);
        y = static_cast<int16_t>((buf[4] << 8) | buf[3]);
        z = static_cast<int16_t>((buf[6] << 8) | buf[5]);
    }
}

int16_t bolt::controller::ICM20948Controller::readTemperature()
{
    selectBank(USER_BANK_0);
    uint8_t buf[2];
    spi_->readRegisters(UB0_TEMP_OUT_H, buf, 2);

    return static_cast<int16_t>((buf[0] << 8) | buf[1]);
}

void bolt::controller::ICM20948Controller::readAll(ICM20948Data &data)
{
    selectBank(USER_BANK_0);

    // Burst read accel (6) + gyro (6) + temp (2) = 14 bytes from 0x2D
    uint8_t buf[14];
    spi_->readRegisters(UB0_ACCEL_XOUT_H, buf, 14);

    data.accel_x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    data.accel_y = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    data.accel_z = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    data.gyro_x = static_cast<int16_t>((buf[6] << 8) | buf[7]);
    data.gyro_y = static_cast<int16_t>((buf[8] << 8) | buf[9]);
    data.gyro_z = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    data.temperature = static_cast<int16_t>((buf[12] << 8) | buf[13]);

    // Magnetometer from EXT_SLV_SENS_DATA (populated by I2C master)
    uint8_t magBuf[8];
    spi_->readRegisters(UB0_EXT_SLV_SENS_DATA_00, magBuf, 8);

    if (magBuf[0] & 0x01)
    {
        data.mag_x = static_cast<int16_t>((magBuf[2] << 8) | magBuf[1]);
        data.mag_y = static_cast<int16_t>((magBuf[4] << 8) | magBuf[3]);
        data.mag_z = static_cast<int16_t>((magBuf[6] << 8) | magBuf[5]);
    }
}

void bolt::controller::ICM20948Controller::selectBank(uint8_t bank)
{
    if (currentBank_ != bank)
    {
        spi_->writeRegister(REG_BANK_SEL, bank);
        currentBank_ = bank;
    }
}

void bolt::controller::ICM20948Controller::reset()
{
    selectBank(USER_BANK_0);
    spi_->writeRegister(UB0_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    currentBank_ = 0xFF;
}

void bolt::controller::ICM20948Controller::configureGyro(GyroFS fs)
{
    selectBank(USER_BANK_2);

    // GYRO_CONFIG_1: DLPF enabled (bit 0), FS_SEL in bits [2:1], DLPF_CFG = 1 (bits [5:3])
    uint8_t config = 0x01 | static_cast<uint8_t>(fs) | (0x01 << 3);
    spi_->writeRegister(UB2_GYRO_CONFIG_1, config);

    // Sample rate divider = 0 (max rate)
    spi_->writeRegister(UB2_GYRO_SMPLRT_DIV, 0x00);
}

void bolt::controller::ICM20948Controller::configureAccel(AccelFS fs)
{
    selectBank(USER_BANK_2);

    // ACCEL_CONFIG: DLPF enabled (bit 0), FS_SEL in bits [2:1], DLPF_CFG = 1 (bits [5:3])
    uint8_t config = 0x01 | static_cast<uint8_t>(fs) | (0x01 << 3);
    spi_->writeRegister(UB2_ACCEL_CONFIG, config);

    // Sample rate divider = 0 (max rate)
    spi_->writeRegister(UB2_ACCEL_SMPLRT_DIV_1, 0x00);
    spi_->writeRegister(UB2_ACCEL_SMPLRT_DIV_2, 0x00);
}

void bolt::controller::ICM20948Controller::configureMag(MagMode mode)
{
    // Reset AK09916
    i2cSlvWrite(AK09916_I2C_ADDR, AK09916_CNTL3, 0x01);
    HAL_Delay(50);

    // Set measurement mode
    i2cSlvWrite(AK09916_I2C_ADDR, AK09916_CNTL2, static_cast<uint8_t>(mode));
    HAL_Delay(10);
}

void bolt::controller::ICM20948Controller::i2cMasterEnable()
{
    // Enable I2C master
    selectBank(USER_BANK_0);
    uint8_t userCtrl = spi_->readRegister(UB0_USER_CTRL);
    spi_->writeRegister(UB0_USER_CTRL, userCtrl | 0x20);

    // I2C master clock = 400 kHz
    selectBank(USER_BANK_3);
    spi_->writeRegister(UB3_I2C_MST_CTRL, 0x07);
    HAL_Delay(10);
}

void bolt::controller::ICM20948Controller::i2cSlvRead(uint8_t addr, uint8_t reg, uint8_t len)
{
    selectBank(USER_BANK_3);
    spi_->writeRegister(UB3_I2C_SLV0_ADDR, addr | 0x80);
    spi_->writeRegister(UB3_I2C_SLV0_REG, reg);
    spi_->writeRegister(UB3_I2C_SLV0_CTRL, 0x80 | len);
    HAL_Delay(10);
}

void bolt::controller::ICM20948Controller::i2cSlvWrite(uint8_t addr, uint8_t reg, uint8_t value)
{
    selectBank(USER_BANK_3);
    spi_->writeRegister(UB3_I2C_SLV0_ADDR, addr & 0x7F);
    spi_->writeRegister(UB3_I2C_SLV0_REG, reg);
    spi_->writeRegister(UB3_I2C_SLV0_DO, value);
    spi_->writeRegister(UB3_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);
}
