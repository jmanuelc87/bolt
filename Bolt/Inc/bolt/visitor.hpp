#ifndef BOLT_VISITOR_HPP
#define BOLT_VISITOR_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "queues.hpp"
#include "frames.hpp"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"

#include "peripherals.hpp"

extern osMessageQueueId_t queryQueue;

using bolt::controller::Motor_ID;

using bolt::controller::MOTOR_ID_M1;
using bolt::controller::MOTOR_ID_M2;
using bolt::controller::MOTOR_ID_M3;
using bolt::controller::MOTOR_ID_M4;

namespace bolt
{

    static size_t build_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out, size_t out_cap)
    {
        if (len > MAX_PAYLOAD || out_cap < (size_t)(1 + 1 + 1 + len + 2 + 1))
            return 0;
        size_t i = 0;
        out[i++] = SOF;
        out[i++] = type;
        out[i++] = len;
        for (uint8_t k = 0; k < len; ++k)
            out[i++] = payload[k];
        const uint16_t c = crc16_ccitt(&out[1], (size_t)2 + len);
        out[i++] = (uint8_t)(c >> 8);
        out[i++] = (uint8_t)(c & 0xFF);
        out[i++] = EOF_;
        return i;
    }

    static void send_message(const char *data)
    {
        Message m;

        uint16_t len = build_frame(0x01, reinterpret_cast<const uint8_t *>(data), strlen(data), m.data, sizeof(m.data));
        m.size = len;

        osStatus_t s = osMessageQueuePut(queryQueue, &m, 0, 0);
        if (s != osOK)
        {
            // failed message
        }
    }

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
    };
}

#endif /* BOLT_VISITOR_HPP */
