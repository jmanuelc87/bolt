#ifndef BOLT_VISITOR_HPP
#define BOLT_VISITOR_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "frames.hpp"
#include "utils.h"

#include "peripherals.hpp"

extern osMessageQueueId_t queryQueue;

using bolt::controller::Motor_ID;

using bolt::controller::MOTOR_ID_M1;
using bolt::controller::MOTOR_ID_M2;
using bolt::controller::MOTOR_ID_M3;
using bolt::controller::MOTOR_ID_M4;

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
            Motor_ID motor_id = static_cast<Motor_ID>(f.motor);
            switch (motor_id)
            {
            case MOTOR_ID_M1:
                gMotor1->setPulse(f.pulse);
                break;

            case MOTOR_ID_M2:
                gMotor2->setPulse(f.pulse);
                break;

            case MOTOR_ID_M3:
                gMotor2->setPulse(f.pulse);
                break;

            case MOTOR_ID_M4:
                gMotor2->setPulse(f.pulse);
                break;

            default:
                break;
            }

            send_message("OK!");
        }

        virtual void visit(const MotorStopFrame &f)
        {
            Motor_ID motor_id = static_cast<Motor_ID>(f.motor);
            switch (motor_id)
            {
            case MOTOR_ID_M1:
                gMotor1->stop(f.brake);
                break;

            case MOTOR_ID_M2:
                gMotor2->stop(f.brake);
                break;

            case MOTOR_ID_M3:
                gMotor2->stop(f.brake);
                break;

            case MOTOR_ID_M4:
                gMotor2->stop(f.brake);
                break;

            default:
                break;
            }

            send_message("OK!");
        }

        virtual void visit(const PwmServoFrame &f)
        {
            gServo->setAngle(f.angle, f.servo);

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
            int i = 5;

            while (--i > 0 && !gUartServo->isReady())
            {
                vTaskDelay(pdMS_TO_TICKS(2));
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
    };
}

#endif /* BOLT_VISITOR_HPP */
