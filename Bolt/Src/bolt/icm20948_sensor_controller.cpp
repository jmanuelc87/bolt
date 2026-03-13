#include "controller/icm20948_sensor_controller.hpp"

bolt::controller::ICM20948SensorController::ICM20948SensorController(ICM20948Controller *imu, GyroFS gyroFs, AccelFS accelFs)
    : imu_(imu), gyroFs_(gyroFs), accelFs_(accelFs)
{
}

bool bolt::controller::ICM20948SensorController::init()
{
    return imu_->init(gyroFs_, accelFs_);
}

void bolt::controller::ICM20948SensorController::readAll(ICM20948SensorData &data)
{
    ICM20948Data raw;
    imu_->readAll(raw);

    float aScale = accelScale();
    float gScale = gyroScale();

    data.accel_x = raw.accel_x * aScale;
    data.accel_y = raw.accel_y * aScale;
    data.accel_z = raw.accel_z * aScale;
    data.gyro_x  = raw.gyro_x  * gScale;
    data.gyro_y  = raw.gyro_y  * gScale;
    data.gyro_z  = raw.gyro_z  * gScale;
    data.mag_x   = raw.mag_x   * MAG_SCALE;
    data.mag_y   = raw.mag_y   * MAG_SCALE;
    data.mag_z   = raw.mag_z   * MAG_SCALE;

    // ICM-20948 datasheet: TEMP_degC = (TEMP_OUT - 21) / 333.87 + 21
    data.temperature = (raw.temperature - 21.0f) / 333.87f + 21.0f;
}

void bolt::controller::ICM20948SensorController::readAccel(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;
    imu_->readAccel(rx, ry, rz);

    float scale = accelScale();
    x = rx * scale;
    y = ry * scale;
    z = rz * scale;
}

void bolt::controller::ICM20948SensorController::readGyro(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;
    imu_->readGyro(rx, ry, rz);

    float scale = gyroScale();
    x = rx * scale;
    y = ry * scale;
    z = rz * scale;
}

void bolt::controller::ICM20948SensorController::readMag(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;
    imu_->readMag(rx, ry, rz);

    x = rx * MAG_SCALE;
    y = ry * MAG_SCALE;
    z = rz * MAG_SCALE;
}

float bolt::controller::ICM20948SensorController::readTemperature()
{
    int16_t raw = imu_->readTemperature();
    return (raw - 21.0f) / 333.87f + 21.0f;
}

float bolt::controller::ICM20948SensorController::accelScale() const
{
    switch (accelFs_)
    {
    case AccelFS::G_2:  return 1.0f / 16384.0f;
    case AccelFS::G_4:  return 1.0f / 8192.0f;
    case AccelFS::G_8:  return 1.0f / 4096.0f;
    case AccelFS::G_16: return 1.0f / 2048.0f;
    default:            return 1.0f / 2048.0f;
    }
}

float bolt::controller::ICM20948SensorController::gyroScale() const
{
    switch (gyroFs_)
    {
    case GyroFS::DPS_250:  return 1.0f / 131.0f;
    case GyroFS::DPS_500:  return 1.0f / 65.5f;
    case GyroFS::DPS_1000: return 1.0f / 32.8f;
    case GyroFS::DPS_2000: return 1.0f / 16.4f;
    default:               return 1.0f / 16.4f;
    }
}
