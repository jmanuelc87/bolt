#ifndef BOLT_ENCODER_CONTROLLER_HPP
#define BOLT_ENCODER_CONTROLLER_HPP

#include "interface/timer_interface.hpp"

using bolt::timer::CountAsyncTimerPort;
using bolt::timer::CountSyncTimerPort;

namespace bolt
{
    namespace controller
    {
        class EncoderController
        {
        public:
            EncoderController(CountAsyncTimerPort *sampler,
                              CountSyncTimerPort *port1,
                              CountSyncTimerPort *port2,
                              CountSyncTimerPort *port3,
                              CountSyncTimerPort *port4) : sampler_(sampler), port1_(port1), port2_(port2), port3_(port3), port4_(port4)
            {
                timPeriodCountElapsed = [this]()
                {
                    if (++enc_div_ >= 10)
                    {
                        enc_div_ = 0;

                        for (uint8_t i = 0; i < 4; i++)
                        {
                            uint32_t diff = port1_->getCount();

                            enc_pos_counts[i] += diff;
                            enc_diff_last[i] = diff;

                            vel_cps[i] = (1.0f - alpha) * vel_cps[i] + alpha * (float)enc_diff_last[i];
                        }
                    }
                };

                sampler_->add(timPeriodCountElapsed);
            }

            ~EncoderController() {}

            float getCPS(uint8_t id)
            {
                return vel_cps[id] * 10000.0f;
            }

            float getRPM(uint8_t id)
            {
                return getCPS(id) * 60.f / ENCODER_CPR;
            }

            int32_t getCounts(uint8_t id)
            {
                return enc_pos_counts[id];
            }

        private:
            CountAsyncTimerPort *sampler_;
            CountSyncTimerPort *port1_;
            CountSyncTimerPort *port2_;
            CountSyncTimerPort *port3_;
            CountSyncTimerPort *port4_;

            uint32_t enc_pos_counts[4] = {0};
            uint32_t enc_diff_last[4];

            float vel_cps[4] = {0.0f};
            const float alpha = 0.2f;

            const uint16_t ENCODER_CPR = 1320.0f * 4.0f;

            uint8_t enc_div_ = 0;

            std::function<void()> timPeriodCountElapsed;
        };
    }
}

#endif /* BOLT_ENCODER_CONTROLLER_HPP */
