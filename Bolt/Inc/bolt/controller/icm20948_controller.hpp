#ifndef BOLT_ICM20948_CONTROLLER_HPP
#define BOLT_ICM20948_CONTROLLER_HPP

#include <cstdint>

#include "interface/spi_interface.hpp"

namespace bolt
{
    namespace controller
    {
        // ICM20948 WHO_AM_I
        static constexpr uint8_t ICM20948_WHO_AM_I_VALUE = 0xEA;

        // Bank select register (shared across all banks)
        static constexpr uint8_t REG_BANK_SEL = 0x7F;

        // ---- User Bank 0 ----
        static constexpr uint8_t UB0_WHO_AM_I = 0x00;
        static constexpr uint8_t UB0_USER_CTRL = 0x03;
        static constexpr uint8_t UB0_LP_CONFIG = 0x05;
        static constexpr uint8_t UB0_PWR_MGMT_1 = 0x06;
        static constexpr uint8_t UB0_PWR_MGMT_2 = 0x07;
        static constexpr uint8_t UB0_INT_PIN_CFG = 0x0F;
        static constexpr uint8_t UB0_INT_ENABLE = 0x10;
        static constexpr uint8_t UB0_INT_ENABLE_1 = 0x11;
        static constexpr uint8_t UB0_INT_STATUS = 0x19;
        static constexpr uint8_t UB0_INT_STATUS_1 = 0x1A;
        static constexpr uint8_t UB0_ACCEL_XOUT_H = 0x2D;
        static constexpr uint8_t UB0_ACCEL_XOUT_L = 0x2E;
        static constexpr uint8_t UB0_ACCEL_YOUT_H = 0x2F;
        static constexpr uint8_t UB0_ACCEL_YOUT_L = 0x30;
        static constexpr uint8_t UB0_ACCEL_ZOUT_H = 0x31;
        static constexpr uint8_t UB0_ACCEL_ZOUT_L = 0x32;
        static constexpr uint8_t UB0_GYRO_XOUT_H = 0x33;
        static constexpr uint8_t UB0_GYRO_XOUT_L = 0x34;
        static constexpr uint8_t UB0_GYRO_YOUT_H = 0x35;
        static constexpr uint8_t UB0_GYRO_YOUT_L = 0x36;
        static constexpr uint8_t UB0_GYRO_ZOUT_H = 0x37;
        static constexpr uint8_t UB0_GYRO_ZOUT_L = 0x38;
        static constexpr uint8_t UB0_TEMP_OUT_H = 0x39;
        static constexpr uint8_t UB0_TEMP_OUT_L = 0x3A;
        static constexpr uint8_t UB0_EXT_SLV_SENS_DATA_00 = 0x3B;
        static constexpr uint8_t UB0_DATA_RDY_STATUS = 0x74;

        // ---- User Bank 2 ----
        static constexpr uint8_t UB2_GYRO_SMPLRT_DIV = 0x00;
        static constexpr uint8_t UB2_GYRO_CONFIG_1 = 0x01;
        static constexpr uint8_t UB2_GYRO_CONFIG_2 = 0x02;
        static constexpr uint8_t UB2_ODR_ALIGN_EN = 0x09;
        static constexpr uint8_t UB2_ACCEL_SMPLRT_DIV_1 = 0x10;
        static constexpr uint8_t UB2_ACCEL_SMPLRT_DIV_2 = 0x11;
        static constexpr uint8_t UB2_ACCEL_CONFIG = 0x14;
        static constexpr uint8_t UB2_ACCEL_CONFIG_2 = 0x15;

        // ---- User Bank 3 ----
        static constexpr uint8_t UB3_I2C_MST_ODR_CONFIG = 0x00;
        static constexpr uint8_t UB3_I2C_MST_CTRL = 0x01;
        static constexpr uint8_t UB3_I2C_MST_DELAY_CTRL = 0x02;
        static constexpr uint8_t UB3_I2C_SLV0_ADDR = 0x03;
        static constexpr uint8_t UB3_I2C_SLV0_REG = 0x04;
        static constexpr uint8_t UB3_I2C_SLV0_CTRL = 0x05;
        static constexpr uint8_t UB3_I2C_SLV0_DO = 0x06;

        // AK09916 magnetometer registers
        static constexpr uint8_t AK09916_I2C_ADDR = 0x0C;
        static constexpr uint8_t AK09916_WHO_AM_I = 0x01;
        static constexpr uint8_t AK09916_WHO_AM_I_VALUE = 0x09;
        static constexpr uint8_t AK09916_ST1 = 0x10;
        static constexpr uint8_t AK09916_XOUT_L = 0x11;
        static constexpr uint8_t AK09916_ST2 = 0x18;
        static constexpr uint8_t AK09916_CNTL2 = 0x31;
        static constexpr uint8_t AK09916_CNTL3 = 0x32;

        // Bank IDs
        static constexpr uint8_t USER_BANK_0 = 0x00;
        static constexpr uint8_t USER_BANK_1 = 0x10;
        static constexpr uint8_t USER_BANK_2 = 0x20;
        static constexpr uint8_t USER_BANK_3 = 0x30;

        // Gyroscope full-scale select
        enum class GyroFS : uint8_t
        {
            DPS_250 = 0x00,
            DPS_500 = 0x02,
            DPS_1000 = 0x04,
            DPS_2000 = 0x06
        };

        // Accelerometer full-scale select
        enum class AccelFS : uint8_t
        {
            G_2 = 0x00,
            G_4 = 0x02,
            G_8 = 0x04,
            G_16 = 0x06
        };

        // AK09916 operation mode
        enum class MagMode : uint8_t
        {
            POWER_DOWN = 0x00,
            SINGLE = 0x01,
            CONTINUOUS_10HZ = 0x02,
            CONTINUOUS_20HZ = 0x04,
            CONTINUOUS_50HZ = 0x06,
            CONTINUOUS_100HZ = 0x08
        };

        struct ICM20948Data
        {
            int16_t accel_x;
            int16_t accel_y;
            int16_t accel_z;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
            int16_t mag_x;
            int16_t mag_y;
            int16_t mag_z;
            int16_t temperature;
        };

        class ICM20948Controller
        {
        public:
            ICM20948Controller(bolt::spi::SpiSyncPort *spi);
            ~ICM20948Controller() {}

            bool init(GyroFS gyroFs = GyroFS::DPS_2000, AccelFS accelFs = AccelFS::G_16);
            bool isConnected();

            void readAccel(int16_t &x, int16_t &y, int16_t &z);
            void readGyro(int16_t &x, int16_t &y, int16_t &z);
            void readMag(int16_t &x, int16_t &y, int16_t &z);
            int16_t readTemperature();
            void readAll(ICM20948Data &data);

        private:
            bolt::spi::SpiSyncPort *spi_;
            uint8_t currentBank_;

            void selectBank(uint8_t bank);
            void reset();
            void configureGyro(GyroFS fs);
            void configureAccel(AccelFS fs);
            void configureMag(MagMode mode);

            void i2cMasterEnable();
            void i2cSlvRead(uint8_t addr, uint8_t reg, uint8_t len);
            void i2cSlvWrite(uint8_t addr, uint8_t reg, uint8_t value);
        };
    }
}

#endif /* BOLT_ICM20948_CONTROLLER_HPP */
