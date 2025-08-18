#include "interface/pin_interface.hpp"

bolt::pin::GpioOutputPin::~GpioOutputPin() = default;


void bolt::pin::GpioOutputPin::setHigh()
{
    HAL_GPIO_WritePin(this->port_, this->pin_, GPIO_PIN_SET);
}

void bolt::pin::GpioOutputPin::setLow()
{
    HAL_GPIO_WritePin(this->port_, this->pin_, GPIO_PIN_RESET);
}

void bolt::pin::GpioOutputPin::toggle()
{
    HAL_GPIO_TogglePin(this->port_, this->pin_);
}