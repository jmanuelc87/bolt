#ifndef BOLT_PERIPHERAL_CONTROLLERS_HPP
#define BOLT_PERIPHERAL_CONTROLLERS_HPP

#include "interface/pin_interface.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

namespace bolt
{
    namespace controller
    {
        class LedController
        {
        public:
            LedController(OutputPin &ledPin) : ledPin_(ledPin) {};
            void blink(int16_t numBlinks, uint16_t delayMs);

        private:
            OutputPin &ledPin_;
        };

        class BeepController
        {
        public:
            BeepController(OutputPin &beepPin) : beepPin_(beepPin) {};
            void on();
            void off();

        private:
            OutputPin &beepPin_;
        };
    }
}

#endif /* BOLT_PERIPHERAL_CONTROLLERS_HPP */
