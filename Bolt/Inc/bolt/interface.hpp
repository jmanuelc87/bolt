#ifndef BOLT_INTERFACE_HPP
#define BOLT_INTERFACE_HPP

#include <functional>
#include <cstdint>

#include "stm32f1xx_hal.h"

namespace bolt
{
    class OutputPin
    {
    public:
        virtual void setHigh() = 0;
        virtual void setLow() = 0;
        virtual void toggle() = 0;
        virtual ~OutputPin() = default;
    };

    class SerialPort
    {
    public:
        virtual void transmit(const uint8_t *data, size_t size) = 0;
        virtual void receive(uint8_t *buffer, size_t size) = 0;
    };

    class AsyncSerialPort
    {
    public:
        virtual int transmit(const uint8_t *data, uint16_t size) = 0;
    };

    class CanBus
    {
    public:
        virtual bool sendMessage(uint32_t id, const uint8_t *data, uint8_t len) = 0;
    };

    class Timer
    {
    };

    class PWMTimer : public Timer
    {
    public:
        virtual void setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4) = 0;
    };

    class CountTimer : public Timer
    {
    public:
        CountTimer(TIM_HandleTypeDef *htim, uint32_t channels) : htim_(htim), channels_(channels) {}

        virtual uint32_t getCount() = 0;

    protected:
        TIM_HandleTypeDef *htim_;
        uint32_t channels_;
    };
}

#endif /* BOLT_INTERFACE_HPP */
