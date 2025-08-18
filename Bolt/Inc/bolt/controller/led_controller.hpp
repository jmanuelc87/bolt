#ifndef BOLT_LED_CONTROLLER_HPP
#define BOLT_LED_CONTROLLER_HPP

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
            LedController(OutputPin& ledPin) : ledPin_(ledPin) {};
            void blink(int16_t numBlinks, uint16_t delayMs);
        private:
            OutputPin& ledPin_;
        };
    }
}

#endif /* BOLT_LED_CONTROLLER_HPP */
