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
            send_message("OK!");
        }

        virtual void visit(const MotorMoveFrame &f)
        {
            gMotorController->setSpeed(f.motor, f.pulse);
            send_message("OK!");
        }

        virtual void visit(const MotorStopFrame &f)
        {
            gMotorController->stop(f.motor, f.brake);
            send_message("OK!");
        }

        virtual void visit(const PwmServoFrame &f)
        {
            send_message("OK!");
        }

        virtual void visit(const UartServoFrame &f)
        {
            gUartServo->setControl(f.servo, f.pulse, f.time);
            send_message("OK!");
        }

        virtual void visit(const UartServoGetAngleFrame &f)
        {
            gUartServo->setControlAngle(f.servo);
            int i = 15;

            while (--i > 0 && !gUartServo->isReady())
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (i > 0)
            {
                uint16_t angle = gUartServo->getAngle();
                char str[10] = "";

                sprintf(str, "%d", angle);
                send_message(str);
            }
            else
            {
                send_message("Err");
            }
        }

        virtual void visit(const EncoderGetValuesFrame &f)
        {
            float rpm = gEncoderController->getRPM(f.motor);
            send_message(rpm);
        }
    };
}

#endif /* BOLT_VISITOR_HPP */
