#ifndef BOLT_PWM_INTERFACE_HPP
#define BOLT_PWM_INTERFACE_HPP

#include <unordered_map>

#include "stm32f1xx_hal.h"

#include "interface.hpp"

#define MOTOR_IGNORE_PULSE (1600)
#define MOTOR_MAX_PULSE (3600)

namespace bolt
{
    namespace pwm
    {
        typedef enum
        {
            MOTOR_ID_M1 = 0,
            MOTOR_ID_M2,
            MOTOR_ID_M3,
            MOTOR_ID_M4,
            MAX_MOTOR
        } Motor_ID;

        class PWMTimerInterface : public bolt::PWMTimer
        {
        public:
            PWMTimerInterface(TIM_TypeDef *tim, Motor_ID motorId) : tim_(tim), motorId_(motorId) {};
            ~PWMTimerInterface() {};

            bool setPulse(int16_t pulse) override;

        private:
            TIM_TypeDef *tim_;
            Motor_ID motorId_;

            int16_t ignore_dead_zone(int16_t pulse);
        };
    }
}

#endif /* BOLT_PWM_INTERFACE_HPP */
