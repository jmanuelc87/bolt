#ifndef BOLT_SERVO_CONTROLLER_HPP
#define BOLT_SERVO_CONTROLLER_HPP

#include <unordered_map>

#include "interface.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

namespace bolt
{
    namespace controller
    {
        class ServoController : public bolt::PWMTimer
        {
        public:
            ServoController(TIM_HandleTypeDef *htim, OutputPin &servo1, OutputPin &servo2, OutputPin &servo3, OutputPin &servo4) : htim_(htim), servo1_(servo1), servo2_(servo2), servo3_(servo3), servo4_(servo4)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    this->g_num_angle[i] = this->PwmServo_Angle_To_Pulse(90);
                }

                HAL_StatusTypeDef s = HAL_ERROR;
                s = HAL_TIM_RegisterCallback(this->htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID, &C_PeriodElapsed);
                configASSERT(s == HAL_OK);
            };

            ~ServoController()
            {
                HAL_TIM_UnRegisterCallback(this->htim_, HAL_TIM_PERIOD_ELAPSED_CB_ID);
            }

            bool setPulse(int16_t pulse) override;
            bool setAngle(int16_t angle, uint8_t servo_id);
            void periodElapsed();
            uint16_t PwmServo_Angle_To_Pulse(uint8_t angle);

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
            OutputPin &servo1_;
            OutputPin &servo2_;
            OutputPin &servo3_;
            OutputPin &servo4_;

            uint16_t g_num_angle[4];
            uint16_t g_pwm_pulse;

            static void C_PeriodElapsed(TIM_HandleTypeDef *htim);
        };
    }
}

#endif /* BOLT_SERVO_CONTROLLER_HPP */
