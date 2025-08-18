#ifndef BOLT_PIN_INTERFACE_HPP
#define BOLT_PIN_INTERFACE_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"

namespace bolt
{
    namespace pin
    {
        class GpioOutputPin : public bolt::OutputPin
        {
        public:
            GpioOutputPin(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_(pin) {};
            ~GpioOutputPin();

            void setHigh() override;
            void setLow() override;
            void toggle() override;

        private:
            GPIO_TypeDef *port_;
            uint16_t pin_;
        };
    };
} // namespace bolt

#endif /* BOLT_PIN_INTERFACE_HPP */
