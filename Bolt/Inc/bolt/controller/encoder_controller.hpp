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
                    if (++enc_div_ >= 100)
                    {
                        enc_div_ = 0;

                        for (uint8_t i = 1; i <= 4; i++)
                        {
                            uint32_t diff = getCount_(i);

                            enc_pos_counts[i - 1] += diff;
                            enc_diff_last[i - 1] = diff;

                            vel_cps[i - 1] = (1.0f - alpha) * vel_cps[i - 1] + alpha * (float)enc_diff_last[i - 1];
                        }
                    }
                };

                sampler_->add(timPeriodCountElapsed);
            }

            ~EncoderController() {}

            float getCPS(uint8_t id)
            {
                return vel_cps[id - 1] * 100.0f;
            }

            float getRPM(uint8_t id)
            {
                return getCPS(id) * 60.f / ENCODER_CPR;
            }

            int32_t getCounts(uint8_t id)
            {
                return enc_pos_counts[id - 1];
            }

        private:
            CountAsyncTimerPort *sampler_;
            CountSyncTimerPort *port1_;
            CountSyncTimerPort *port2_;
            CountSyncTimerPort *port3_;
            CountSyncTimerPort *port4_;

            int32_t enc_pos_counts[4] = {0, 0, 0, 0};
            int32_t enc_diff_last[4] = {0, 0, 0, 0};

            float vel_cps[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            const float alpha = 0.2f;

            const int16_t ENCODER_CPR = 1320.0f * 4.0f;

            uint8_t enc_div_ = 0;

            std::function<void()> timPeriodCountElapsed;

            uint32_t getCount_(uint8_t port_id)
            {
                switch (port_id)
                {
                case 1:
                    return port1_->getCount();

                case 2:
                    return port2_->getCount();

                case 3:
                    return port3_->getCount();

                case 4:
                    return port4_->getCount();
                }

                return 0;
            }
        };
    }
}

#endif /* BOLT_ENCODER_CONTROLLER_HPP */
