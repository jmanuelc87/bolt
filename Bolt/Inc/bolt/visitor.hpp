#ifndef BOLT_VISITOR_HPP
#define BOLT_VISITOR_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "queues.hpp"
#include "frames.hpp"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"

#include "interface/pwm_interface.hpp"

using bolt::pwm::MOTOR_ID_M1;
using bolt::pwm::MOTOR_ID_M2;
using bolt::pwm::MOTOR_ID_M3;
using bolt::pwm::MOTOR_ID_M4;
using bolt::pwm::PWMTimerInterface;

extern osMessageQueueId_t queryQueue;

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

    static PWMTimerInterface motor1(TIM8, MOTOR_ID_M1);
    static PWMTimerInterface motor2(TIM8, MOTOR_ID_M2);
    static PWMTimerInterface motor3(TIM1, MOTOR_ID_M3);
    static PWMTimerInterface motor4(TIM1, MOTOR_ID_M4);

    struct AppVisitor : public FrameVisitor
    {
        // Provide storage or references to your system components here
        // (sensors, config, queues, etc.)

        virtual void visit(const PingFrame &)
        {
            Message m;

            const char *data = "DONE!";

            uint16_t len = build_frame(0x01, reinterpret_cast<const uint8_t *>(data), strlen(data), m.data, sizeof(m.data));
            m.size = len;

            osStatus_t s = osMessageQueuePut(queryQueue, &m, 0, 0);
            if (s != osOK)
            {
                // failed message
            }
        }

        virtual void visit(const SetMotorFrame &f)
        {
            switch (f.key)
            {
            case 1:
                motor1.setPulse(f.value);
                break;

            case 2:
                motor2.setPulse(f.value);
                break;

            case 3:
                motor3.setPulse(f.value);
                break;

            case 4:
                motor4.setPulse(f.value);
                break;

            default:
                break;
            }
        }
    };
}

#endif /* BOLT_VISITOR_HPP */
