#ifndef BOLT_ENCODER_CONTROLLER_HPP
#define BOLT_ENCODER_CONTROLLER_HPP

#include <array>
#include <cstdint>

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
                              CountSyncTimerPort *port4)
                : sampler_(sampler),
                  ports_{port1, port2, port3, port4}
            {
                sampler_->timElapsedCompleteCallback = [this]()
                {
                    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
                    {
                        int32_t diff = ports_[i]->count();
                        enc_pos_counts_[i] += diff;
                        vel_cps_[i] = (1.0f - ALPHA) * vel_cps_[i] + ALPHA * static_cast<float>(diff);
                    }
                };
            }

            ~EncoderController() = default;

            float getCPS(uint8_t id)
            {
                return vel_cps_[id - 1] * 100.0f;
            }

            float getRPM(uint8_t id)
            {
                return getCPS(id) * 60.0f / ENCODER_CPR;
            }

            int32_t getCounts(uint8_t id)
            {
                return enc_pos_counts_[id - 1];
            }

        private:
            static constexpr uint8_t NUM_ENCODERS = 4;
            static constexpr float ALPHA = 0.2f;
            static constexpr float ENCODER_CPR = 2464.0f;

            ProcessAsyncTimerPort *sampler_;

            std::array<CountSyncTimerPort *, NUM_ENCODERS> ports_;

            std::array<int32_t, NUM_ENCODERS> enc_pos_counts_{};
            std::array<float, NUM_ENCODERS> vel_cps_{};
        };
    }
}

#endif /* BOLT_ENCODER_CONTROLLER_HPP */
