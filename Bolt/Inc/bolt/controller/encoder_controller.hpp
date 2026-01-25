#ifndef BOLT_ENCODER_CONTROLLER_HPP
#define BOLT_ENCODER_CONTROLLER_HPP

#include "interface/timer_interface.hpp"

using bolt::timer::CountSyncTimerPort;
using bolt::timer::ProcessAsyncTimerPort;

namespace bolt
{
    namespace controller
    {
        class EncoderController
        {
        public:
            EncoderController(ProcessAsyncTimerPort *sampler,
                              CountSyncTimerPort *port1,
                              CountSyncTimerPort *port2,
                              CountSyncTimerPort *port3,
                              CountSyncTimerPort *port4) : sampler_(sampler)
            {
                setPorts(port1, port2, port3, port4);

                sampler_->timElapsedCompleteCallback = [this]()
                {
                    for (uint8_t i = 0; i < 4; i++)
                    {
                        int32_t diff = port[i]->count();
                        enc_pos_counts[i] += diff;
                        enc_diff_last[i] = diff;
                        vel_cps[i] = (1.0f - alpha) * vel_cps[i] + alpha * (float)enc_diff_last[i];
                    }
                };
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
            ProcessAsyncTimerPort *sampler_;
            CountSyncTimerPort *port[4];

            int32_t enc_pos_counts[4] = {0, 0, 0, 0};
            int32_t enc_diff_last[4] = {0, 0, 0, 0};

            float vel_cps[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            const float alpha = 0.2f;

            const int16_t ENCODER_CPR = 2464.0f;

            uint8_t enc_div_ = 0;

            void setPorts(CountSyncTimerPort *port1, CountSyncTimerPort *port2, CountSyncTimerPort *port3, CountSyncTimerPort *port4)
            {
                port[0] = port1;
                port[1] = port2;
                port[2] = port3;
                port[3] = port4;
            };
        };
    }
}

#endif /* BOLT_ENCODER_CONTROLLER_HPP */
