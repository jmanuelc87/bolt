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
            PWMSyncTimerPort(TIM_HandleTypeDef *htim, int32_t chan_pwm[4], int32_t chan_pwmn[4]) : PWMTimerPort(htim)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    if (chan_pwm[i] != -1)
                    {
                        HAL_TIM_PWM_Start(htim_, chan_pwm[i]);
                    }

                    if (chan_pwmn[i] != -1)
                    {
                        HAL_TIMEx_PWMN_Start(htim_, chan_pwmn[i]);
                    }
                }
            }

            ~PWMSyncTimerPort() {}

            void setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4) override;
        };

        class CountSyncTimerPort : public bolt::CountTimer
        {
        public:
            CountSyncTimerPort(TIM_HandleTypeDef *htim, uint64_t channels) : CountTimer(htim, channels)
            {
                HAL_TIM_Encoder_Start(htim_, channels_);
                __HAL_TIM_SET_COUNTER(htim_, 0);
            }

            virtual int32_t count() override;

        protected:
            int32_t last_ = 0;
            int32_t now_ = 0;

            const int32_t ENC_PERIOD = 0x10000000U;
            const int32_t ENC_HALF_PERIOD = (ENC_PERIOD / 2);
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
                    for (uint16_t i = 0; i < callbacks.size(); i++)
                    {
                        if (++timers[i] >= callbacks[i].second)
                        {
                            callbacks[i].first();
                            timers[i] = 0;
                        }
                    }
                };

                HAL_TIM_Base_Start_IT(htim_);
            }

            ~CountAsyncTimerPort()
            {
                HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>::unregisterCallbacks(htim_);
            }

            void add(CountAsyncTimerCallback cb, uint16_t time)
            {
                auto p = std::make_pair(cb, time);
                callbacks.push_back(p);
                timers.push_back(0);
            }

            virtual int32_t count() override;

            friend class HandleRegistry<CountAsyncTimerPort, TIM_HandleTypeDef>;

        private:
            std::vector<uint16_t> timers;
            std::vector<std::pair<CountAsyncTimerCallback, uint16_t>> callbacks;
            std::function<void()> timElapsedCompleteCallback;
        };

        typedef struct __PROC_HandleTypeDef
        {
            uint16_t timer;
            uint16_t counter;
        } PROC_HandleTypeDef;

        class ProcessAsyncTimerPort
        {
        public:
            ProcessAsyncTimerPort(PROC_HandleTypeDef *p) : proc(p)
            {
                HandleRegistry<ProcessAsyncTimerPort, PROC_HandleTypeDef>::registry().insert({p, this});
            };

            ~ProcessAsyncTimerPort()
            {
                HandleRegistry<ProcessAsyncTimerPort, PROC_HandleTypeDef>::registry().erase(proc);
            };

            friend class HandleRegistry<ProcessAsyncTimerPort, PROC_HandleTypeDef>;

            std::function<void()> timElapsedCompleteCallback;

        private:
            PROC_HandleTypeDef *proc;
        };
    }
}

#endif /* BOLT_TIMER_INTERFACE_HPP */
