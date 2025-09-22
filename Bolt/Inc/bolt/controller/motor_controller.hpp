#ifndef BOLT_MOTOR_CONTROLLER_HPP
#define BOLT_MOTOR_CONTROLLER_HPP

#include "tim.h"
#include "stm32f1xx_hal.h"

#include "interface.hpp"

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

namespace bolt
{
    namespace controller
    {

        typedef enum
        {
            MOTOR_ID_M1 = 1,
            MOTOR_ID_M2,
            MOTOR_ID_M3,
            MOTOR_ID_M4,
            MAX_MOTOR
        } Motor_ID;

        class MotorController : public bolt::PWMTimer
        {
        public:
            MotorController(Motor_ID motor_id) : motor_id_(motor_id)
            {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
                HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

                // After MX_TIM8_Init() and GPIO init:
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
            };
            ~MotorController() {};

            bool setPulse(int16_t pulse) override;
            bool stop(uint8_t brake);

        private:
            Motor_ID motor_id_;
            int16_t ignore_dead_zone(int16_t pulse);
        };
    }
}

#endif /* BOLT_MOTOR_CONTROLLER_HPP */
