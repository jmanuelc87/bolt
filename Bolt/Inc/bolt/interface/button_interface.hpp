#ifndef BOLT_BUTTON_INTERFACE_HPP
#define BOLT_BUTTON_INTERFACE_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"

namespace bolt
{
    namespace pin
    {
        class GpioInputPin : public bolt::InputPin
        {
        public:
            GpioInputPin(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_(pin) {}
            ~GpioInputPin() = default;

            bool read() override
            {
                return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
            }

        private:
            GPIO_TypeDef *port_;
            uint16_t pin_;
        };
    }
}

#endif /* BOLT_BUTTON_INTERFACE_HPP */
