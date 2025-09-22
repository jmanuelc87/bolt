#ifndef BOLT_SERVO_CONTROLLER_HPP
#define BOLT_SERVO_CONTROLLER_HPP

#include <unordered_map>
#include <cstdint>

#include "interface/pin_interface.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

using bolt::pin::GpioOutputPin;

namespace bolt
{
    namespace controller
    {
        // how often TIM7 interrupt fires (us)
        static constexpr uint16_t STEP_US = 50;
        static constexpr uint16_t FRAME_US = 20000; // 20ms frame
        static constexpr uint16_t FRAME_TICKS = FRAME_US / STEP_US;

        // per-channel countdowns
        static volatile uint16_t ch_ticks[4];
        static volatile uint16_t frame_ticks = FRAME_TICKS;

        class ServoController : public bolt::PWMTimer
        {
        public:
            ServoController(TIM_HandleTypeDef *htim) : htim_(htim)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    this->setAngle(90, i);
                }

                HAL_StatusTypeDef s = HAL_ERROR;
                s = HAL_TIM_RegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &C_PeriodElapsed);
                configASSERT(s == HAL_OK);

                s = HAL_TIM_Base_Start_IT(htim_);
                configASSERT(s == HAL_OK);
            };

            ~ServoController()
            {
                HAL_TIM_UnRegisterCallback(htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID);
                HAL_TIM_UnRegisterCallback(htim_, HAL_TIM_OC_DELAY_ELAPSED_CB_ID);
            }

            bool setPulse(int16_t pulse) override;
            bool setAngle(int16_t angle, uint8_t servo_id);
            void periodElapsed();
            float PwmServo_Angle_To_Us(uint8_t angle);

            static std::unordered_map<TIM_HandleTypeDef *, ServoController *> &registry()
            {
                static std::unordered_map<TIM_HandleTypeDef *, ServoController *> r;
                return r;
            }

            static ServoController *from(TIM_HandleTypeDef *h)
            {
                auto it = registry().find(h);
                return (it == registry().end()) ? nullptr : it->second;
            }

        private:
            TIM_HandleTypeDef *htim_;

            float g_pwm_pulse[4];

            static void C_PeriodElapsed(TIM_HandleTypeDef *htim);
        };
    }
}

#endif /* BOLT_SERVO_CONTROLLER_HPP */
