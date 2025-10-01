#ifndef BOLT_TIMER_INTERFACE_HPP
#define BOLT_TIMER_INTERFACE_HPP

#include <cstdint>
#include <vector>

#include "interface.hpp"
#include "registry/handle_registry.hpp"
#include "stm32f1xx_hal.h"

using bolt::registry::HandleRegistry;
using CountAsyncTimerCallback = std::function<void()>;

namespace bolt
{
    namespace timer
    {
        class PWMTimerPort : public bolt::PWMTimer
        {
        public:
            PWMTimerPort(TIM_HandleTypeDef *htim) : htim_(htim) {}

            virtual void setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4) = 0;

        protected:
            TIM_HandleTypeDef *htim_;
        };

        class PWMAsyncTimerPort : public PWMTimerPort
        {
        public:
            PWMAsyncTimerPort(TIM_HandleTypeDef *htim) : PWMTimerPort(htim) {}
            ~PWMAsyncTimerPort() {}
        };

        class PWMSyncTimerPort : public PWMTimerPort
        {
        public:
            PWMSyncTimerPort(TIM_HandleTypeDef *htim, uint32_t chan_pwm, uint32_t chan_pwmn) : PWMTimerPort(htim)
            {
                if (chan_pwm != 0)
                {
                    HAL_TIM_PWM_Start(htim_, chan_pwm);
                }

                if (chan_pwmn != 0)
                {
                    HAL_TIMEx_PWMN_Start(htim_, chan_pwmn);
                }
            }

            ~PWMSyncTimerPort() {}

            void setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4) override;
        };

        class CountSyncTimerPort : public bolt::CountTimer
        {
        public:
            CountSyncTimerPort(TIM_HandleTypeDef *htim, uint32_t channels) : CountTimer(htim, channels)
            {
                HAL_TIM_Encoder_Start(htim_, channels_);
                __HAL_TIM_SET_COUNTER(htim_, 0);
            }

            virtual uint32_t getCount() override;

        protected:
            uint32_t last_ = 0;
            uint32_t now_;

            const uint32_t ENC_PERIOD = 0x10000U;
            const uint32_t ENC_HALF = ENC_PERIOD / 2;
        };

        class CountAsyncTimerPort : public bolt::CountTimer
        {
        public:
            CountAsyncTimerPort(TIM_HandleTypeDef *sampler) : CountTimer(sampler, 0)
            {
                HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>::registerCallbacks(htim_);
                HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>::registry().insert({htim_, this});

                timElapsedCompleteCallback = [this]()
                {
                    for (auto &cb : callbacks)
                    {
                        cb();
                    }
                };

                HAL_TIM_Base_Start_IT(htim_);
            }

            ~CountAsyncTimerPort()
            {
                HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>::unregisterCallbacks(htim_);
            }

            void add(CountAsyncTimerCallback cb)
            {
                callbacks.push_back(cb);
            }

            friend class HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>;

        private:
            std::vector<CountAsyncTimerCallback> callbacks;
            std::function<void()> timElapsedCompleteCallback;
        };
    }
}

#endif /* BOLT_TIMER_INTERFACE_HPP */
