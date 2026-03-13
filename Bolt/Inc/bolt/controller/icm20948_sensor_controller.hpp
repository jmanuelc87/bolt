#ifndef BOLT_ICM20948_SENSOR_CONTROLLER_HPP
#define BOLT_ICM20948_SENSOR_CONTROLLER_HPP

#include "controller/icm20948_controller.hpp"

namespace bolt
{
    namespace controller
    {
        struct ICM20948SensorData
        {
            float accel_x;    // g
            float accel_y;    // g
            float accel_z;    // g
            float gyro_x;     // deg/s
            float gyro_y;     // deg/s
            float gyro_z;     // deg/s
            float mag_x;      // µT
            float mag_y;      // µT
            float mag_z;      // µT
            float temperature; // °C
        };

        class ICM20948SensorController
        {
        public:
            ICM20948SensorController(ICM20948Controller *imu, GyroFS gyroFs = GyroFS::DPS_2000, AccelFS accelFs = AccelFS::G_16);
            ~ICM20948SensorController() = default;

            bool init();

            void readAll(ICM20948SensorData &data);
            void readAccel(float &x, float &y, float &z);
            void readGyro(float &x, float &y, float &z);
            void readMag(float &x, float &y, float &z);
            float readTemperature();

        private:
            ICM20948Controller *imu_;
            GyroFS gyroFs_;
            AccelFS accelFs_;

            float accelScale() const;
            float gyroScale() const;

            // AK09916 fixed sensitivity: 0.15 µT/LSB
            static constexpr float MAG_SCALE = 0.15f;
        };
    }
}

#endif /* BOLT_ICM20948_SENSOR_CONTROLLER_HPP */
