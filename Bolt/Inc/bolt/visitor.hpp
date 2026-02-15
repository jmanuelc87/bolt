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
            bolt::controller::ICM20948Data data{};
            gImuController->readAll(data);

            uint8_t payload[20];
            payload[0] = (uint8_t)(data.accel_x >> 8);
            payload[1] = (uint8_t)(data.accel_x & 0xFF);
            payload[2] = (uint8_t)(data.accel_y >> 8);
            payload[3] = (uint8_t)(data.accel_y & 0xFF);
            payload[4] = (uint8_t)(data.accel_z >> 8);
            payload[5] = (uint8_t)(data.accel_z & 0xFF);
            payload[6] = (uint8_t)(data.gyro_x >> 8);
            payload[7] = (uint8_t)(data.gyro_x & 0xFF);
            payload[8] = (uint8_t)(data.gyro_y >> 8);
            payload[9] = (uint8_t)(data.gyro_y & 0xFF);
            payload[10] = (uint8_t)(data.gyro_z >> 8);
            payload[11] = (uint8_t)(data.gyro_z & 0xFF);
            payload[12] = (uint8_t)(data.mag_x >> 8);
            payload[13] = (uint8_t)(data.mag_x & 0xFF);
            payload[14] = (uint8_t)(data.mag_y >> 8);
            payload[15] = (uint8_t)(data.mag_y & 0xFF);
            payload[16] = (uint8_t)(data.mag_z >> 8);
            payload[17] = (uint8_t)(data.mag_z & 0xFF);
            payload[18] = (uint8_t)(data.temperature >> 8);
            payload[19] = (uint8_t)(data.temperature & 0xFF);

            Message m;
            m.size = build_frame(IMU, payload, 20, m.data, sizeof(m.data));
            osMessageQueuePut(queryQueue, &m, 0, 0);
        }
    };
}

#endif /* BOLT_VISITOR_HPP */
