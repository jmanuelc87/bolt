#ifndef BOLT_MOTOR_CONTROLLER_HPP
#define BOLT_MOTOR_CONTROLLER_HPP

#include <map>

#include "tim.h"
#include "stm32f1xx_hal.h"

#include "interface/timer_interface.hpp"

#define PWM_M1_A TIM8->CCR1
#define PWM_M1_B TIM8->CCR2

#define PWM_M2_A TIM8->CCR3
#define PWM_M2_B TIM8->CCR4

#define PWM_M3_A TIM1->CCR4
#define PWM_M3_B TIM1->CCR1

#define PWM_M4_A TIM1->CCR2
#define PWM_M4_B TIM1->CCR3

#define MOTOR_IGNORE_PULSE (1600)
#define MOTOR_MAX_PULSE (3600)

using bolt::timer::PWMSyncTimerPort;

namespace bolt
{
    namespace controller
    {
        class MotorController
        {
        public:
            MotorController(PWMSyncTimerPort *port1, PWMSyncTimerPort *port2) : port1_(port1), port2_(port2) {}
            ~MotorController() {}

            void setSpeed(uint8_t motor_id, int16_t pulse);
            void stop(uint8_t motor_id, uint8_t brake);

        private:
            PWMSyncTimerPort *port1_;
            PWMSyncTimerPort *port2_;

            uint16_t pulse_motor_[8] = {0};

            const int16_t MAX_MOTOR_PULSE = 3600;

            std::map<int, std::pair<int, int>> motor2ports_ = {
                {0, {4, 5}},
                {1, {7, 6}},
                {2, {3, 0}},
                {3, {1, 2}},
            };

            int16_t ignore_dead_zone(int16_t pulse);
            int16_t clamp(int16_t pulse);
        };
    }
}

#endif /* BOLT_MOTOR_CONTROLLER_HPP */
