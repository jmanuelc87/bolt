#ifndef BOLT_VISITOR_HPP
#define BOLT_VISITOR_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "frames.hpp"
#include "utils.h"

#include "peripherals.hpp"

namespace bolt
{
    struct AppVisitor : public FrameVisitor
    {
        virtual void visit(const PingFrame &)
        {
            send_payload(PONG, "OK!");
        }

        virtual void visit(const MotorSpeedFrame &f)
        {
            gMotorController->setSpeed(f.motor, f.pulse);
        }

        virtual void visit(const MotorStopFrame &f)
        {
            gMotorController->stop(f.motor, f.brake);
        }

        virtual void visit(const PwmServoFrame &f)
        {
            gPwmServo->setAngle(f.servo, f.angle);
        }

        virtual void visit(const UartServoFrame &f)
        {
            gUartServo->setControl(f.servo, f.pulse, f.time);
        }

        virtual void visit(const UartServoGetAngleFrame &f)
        {
            gUartServo->setControlAngle(f.servo);
            int i = 10;

            while (--i > 0 && !gUartServo->isReady())
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            if (i > 0)
            {
                int pwm = gUartServo->getAngle();
                float percent = (pwm - 100) / 3800; // normalize pwm value
                float angle = 180 * percent;        // calculate angle
                send_payload(ANGLE, new float[]{angle}, 1);
            }
            else
            {
                send_payload(ANGLE, new float[]{-1.0}, 1);
            }
        }

        virtual void visit(const EncoderGetValuesFrame &f)
        {
            (void)f;
            float rpm1 = gEncoderController->getRPM(1);
            float rpm2 = gEncoderController->getRPM(2);
            float rpm3 = gEncoderController->getRPM(3);
            float rpm4 = gEncoderController->getRPM(4);

            float array[] = {rpm1, rpm2, rpm3, rpm4};

            send_payload(RPMS, array, 4);
        }

        virtual void visit(const ImuGetValuesFrame &f)
        {
            (void)f;
            bolt::controller::ICM20948SensorData data{};
            gImuSensorController->readAll(data);

            // Encode as scaled big-endian int16_t (20 bytes total):
            //   accel ×1000 → mg  |  gyro ×10 → 0.1 °/s  |  mag ×1 → µT  |  temp ×100 → 0.01 °C
            auto pack = [](uint8_t *dst, int16_t v)
            {
                dst[0] = (uint8_t)(v >> 8);
                dst[1] = (uint8_t)(v & 0xFF);
            };

            uint8_t payload[20];
            pack(&payload[0],  static_cast<int16_t>(data.accel_x     * 1000.0f));
            pack(&payload[2],  static_cast<int16_t>(data.accel_y     * 1000.0f));
            pack(&payload[4],  static_cast<int16_t>(data.accel_z     * 1000.0f));
            pack(&payload[6],  static_cast<int16_t>(data.gyro_x      * 10.0f));
            pack(&payload[8],  static_cast<int16_t>(data.gyro_y      * 10.0f));
            pack(&payload[10], static_cast<int16_t>(data.gyro_z      * 10.0f));
            pack(&payload[12], static_cast<int16_t>(data.mag_x));
            pack(&payload[14], static_cast<int16_t>(data.mag_y));
            pack(&payload[16], static_cast<int16_t>(data.mag_z));
            pack(&payload[18], static_cast<int16_t>(data.temperature * 100.0f));

            Message m;
            m.size = build_frame(IMU, payload, 20, m.data, sizeof(m.data));
            osMessageQueuePut(queryQueue, &m, 0, 0);
        }

        virtual void visit(const PidMotorSetRpmFrame &f)
        {
            uint8_t idx = f.motor - 1;
            if (idx < 4 && gPidMotorController[idx])
            {
                gPidMotorController[idx]->setTargetRPM(f.rpm);
            }
        }

        virtual void visit(const PidMotorStopFrame &f)
        {
            uint8_t idx = f.motor - 1;
            if (idx < 4 && gPidMotorController[idx])
            {
                gPidMotorController[idx]->stop(f.brake);
            }
        }

        virtual void visit(const PidSetGainsFrame &f)
        {
            uint8_t idx = f.motor - 1;
            if (idx < 4 && gPidMotorController[idx])
            {
                gPidMotorController[idx]->setGains(f.kp, f.ki, f.kd);

                if (f.save && gFlashController)
                {
                    uint8_t base = idx * 3;
                    gFlashController->store(static_cast<bolt::controller::FlashKey>(base + 0), f.kp);
                    gFlashController->store(static_cast<bolt::controller::FlashKey>(base + 1), f.ki);
                    gFlashController->store(static_cast<bolt::controller::FlashKey>(base + 2), f.kd);
                }
            }
        }

        virtual void visit(const GetBatteryDataFrame &f)
        {
            (void)f;
            if (gBatteryMonitor)
            {
                float array[] = {gBatteryMonitor->voltage(), gBatteryMonitor->percentage()};
                send_payload(BATTERY, array, 2);
            }
        }
    };
}

#endif /* BOLT_VISITOR_HPP */
