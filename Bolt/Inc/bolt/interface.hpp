#ifndef BOLT_INTERFACE_HPP
#define BOLT_INTERFACE_HPP

#include <functional>
#include <cstdint>

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
}

#endif /* BOLT_INTERFACE_HPP */
