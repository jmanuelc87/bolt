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
            (void)f;
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
    };
}

#endif /* BOLT_VISITOR_HPP */
