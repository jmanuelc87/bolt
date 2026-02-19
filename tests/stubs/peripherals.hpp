#ifndef BOLT_PERIPHERALS_HPP
#define BOLT_PERIPHERALS_HPP

// ---------------------------------------------------------------------------
// Stub peripherals.hpp — replaces Bolt/Inc/peripherals.hpp for host tests.
//
// Searched before the real header because tests/stubs/ appears first in the
// CMake include path list.  Provides the same global pointer names and
// controller types used by visitor.hpp, but with no HAL dependencies.
//
// All mock classes live in their real namespaces so visitor.hpp compiles
// unchanged.  Method signatures match exactly what visitor.hpp calls.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstring>

#include "definitions.hpp"

// ---------------------------------------------------------------------------
// ICM20948Data — defined here instead of the real icm20948_controller.hpp
// so that visitor.hpp's inline "bolt::controller::ICM20948Data data{};" works.
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
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
    }
}

// ---------------------------------------------------------------------------
// Mock MotorController
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class MotorController
        {
        public:
            // Last-call recording
            uint8_t last_setSpeed_motor = 0;
            int16_t last_setSpeed_pulse = 0;
            int     call_count_setSpeed = 0;

            uint8_t last_stop_motor = 0;
            uint8_t last_stop_brake = 0;
            int     call_count_stop = 0;

            void setSpeed(uint8_t motor_id, int16_t pulse)
            {
                last_setSpeed_motor = motor_id;
                last_setSpeed_pulse = pulse;
                ++call_count_setSpeed;
            }

            void stop(uint8_t motor_id, uint8_t brake)
            {
                last_stop_motor = motor_id;
                last_stop_brake = brake;
                ++call_count_stop;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// Mock PWMServoController
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class PWMServoController
        {
        public:
            uint8_t last_servo = 0;
            uint8_t last_angle = 0;
            int     call_count_setAngle = 0;

            void setAngle(uint8_t servo_id, uint8_t angle)
            {
                last_servo = servo_id;
                last_angle = angle;
                ++call_count_setAngle;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// Mock UartServoController
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class UartServoController
        {
        public:
            // setControl recording
            uint8_t  last_setControl_servo = 0;
            uint16_t last_setControl_pulse = 0;
            uint16_t last_setControl_time  = 0;
            int      call_count_setControl = 0;

            // setControlAngle recording
            uint8_t  last_setControlAngle_servo = 0;
            int      call_count_setControlAngle = 0;

            // isReady / getAngle control
            bool     ready_return = false;
            uint16_t angle_return = 0;
            int      call_count_isReady  = 0;
            int      call_count_getAngle = 0;

            void setControl(uint8_t id, uint16_t pulse, uint16_t time)
            {
                last_setControl_servo = id;
                last_setControl_pulse = pulse;
                last_setControl_time  = time;
                ++call_count_setControl;
            }

            void setControlAngle(uint8_t id)
            {
                last_setControlAngle_servo = id;
                ++call_count_setControlAngle;
            }

            bool isReady()
            {
                ++call_count_isReady;
                return ready_return;
            }

            uint16_t getAngle()
            {
                ++call_count_getAngle;
                return angle_return;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// Mock EncoderController
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class EncoderController
        {
        public:
            float rpm_values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            int   call_count_getRPM = 0;

            // Returns rpm_values[id-1] for id in 1..4; 0 otherwise
            float getRPM(uint8_t id)
            {
                ++call_count_getRPM;
                if (id >= 1 && id <= 4)
                    return rpm_values[id - 1];
                return 0.0f;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// Mock ICM20948Controller
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class ICM20948Controller
        {
        public:
            ICM20948Data mock_data  = {};
            int          call_count_readAll = 0;

            void readAll(ICM20948Data &data)
            {
                ++call_count_readAll;
                data = mock_data;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// Mock PIDMotorController
// Visitor calls: setTargetRPM(float), stop(uint8_t), setGains(float,float,float)
// ---------------------------------------------------------------------------
namespace bolt
{
    namespace controller
    {
        class PIDMotorController
        {
        public:
            float   last_setTargetRPM_rpm = 0.0f;
            int     call_count_setTargetRPM = 0;

            uint8_t last_stop_brake = 0;
            int     call_count_stop = 0;

            float   last_setGains_kp = 0.0f;
            float   last_setGains_ki = 0.0f;
            float   last_setGains_kd = 0.0f;
            int     call_count_setGains = 0;

            void setTargetRPM(float rpm)
            {
                last_setTargetRPM_rpm = rpm;
                ++call_count_setTargetRPM;
            }

            void stop(uint8_t brake)
            {
                last_stop_brake = brake;
                ++call_count_stop;
            }

            void setGains(float kp, float ki, float kd)
            {
                last_setGains_kp = kp;
                last_setGains_ki = ki;
                last_setGains_kd = kd;
                ++call_count_setGains;
            }
        };
    }
}

// ---------------------------------------------------------------------------
// FlashController — use real header (header-only, no HAL deps)
// ---------------------------------------------------------------------------
#include "bolt/controller/flash_controller.hpp"

// ---------------------------------------------------------------------------
// Using declarations matching the real peripherals.hpp so visitor.hpp's
// unqualified global pointer types resolve correctly.
// ---------------------------------------------------------------------------
using bolt::controller::UartServoController;
using bolt::controller::EncoderController;
using bolt::controller::ICM20948Controller;
using bolt::controller::MotorController;
using bolt::controller::PIDMotorController;
using bolt::controller::PWMServoController;
using bolt::controller::FlashController;

// ---------------------------------------------------------------------------
// Global pointers — declared inline so they can be included from multiple
// translation units without violating the ODR.
// Test fixtures assign concrete mock instances before exercising the visitor.
// ---------------------------------------------------------------------------
inline MotorController      *gMotorController   = nullptr;
inline PWMServoController   *gPwmServo           = nullptr;
inline UartServoController  *gUartServo          = nullptr;
inline EncoderController    *gEncoderController  = nullptr;
inline ICM20948Controller   *gImuController      = nullptr;
inline PIDMotorController   *gPidMotorController[4] = {nullptr, nullptr, nullptr, nullptr};
inline FlashController      *gFlashController    = nullptr;

#endif /* BOLT_PERIPHERALS_HPP */