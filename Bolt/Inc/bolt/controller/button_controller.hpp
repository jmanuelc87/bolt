#ifndef BOLT_BUTTON_CONTROLLER_HPP
#define BOLT_BUTTON_CONTROLLER_HPP

#include <cstdint>
#include <functional>

#include "interface.hpp"
#include "interface/timer_interface.hpp"

using bolt::timer::ProcessAsyncTimerPort;

namespace bolt
{
    namespace controller
    {
        class ButtonController
        {
        public:
            // debounce_ticks: number of consecutive equal reads (at 5ms/tick) before
            // the logical state changes. Default 4 ticks = 20ms debounce.
            ButtonController(ProcessAsyncTimerPort *sampler,
                             bolt::InputPin *pin,
                             uint8_t debounce_ticks = 4)
                : sampler_(sampler), pin_(pin), debounce_ticks_(debounce_ticks)
            {
                sampler_->timElapsedCompleteCallback = [this]()
                {
                    poll();
                };
            }

            ~ButtonController() = default;

            std::function<void()> onPressed;

        private:
            void poll()
            {
                bool raw = pin_->read();

                if (raw == debounce_state_)
                {
                    debounce_count_ = 0;
                    return;
                }

                if (++debounce_count_ >= debounce_ticks_)
                {
                    debounce_count_ = 0;
                    debounce_state_ = raw;

                    if (raw && onPressed)
                    {
                        onPressed();
                    }
                }
            }

            ProcessAsyncTimerPort *sampler_;
            bolt::InputPin *pin_;
            uint8_t debounce_ticks_;

            bool debounce_state_ = false;
            uint8_t debounce_count_ = 0;
        };
    }
}

#endif /* BOLT_BUTTON_CONTROLLER_HPP */
