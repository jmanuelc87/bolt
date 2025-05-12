#ifndef BOLT_INTERFACE_H
#define BOLT_INTERFACE_H

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
        virtual void transmit(const uint8_t* data, size_t size) = 0;
        virtual void receive(uint8_t* buffer, size_t size) = 0;
    };

    class AsyncSerialPort
    {
    public:
        virtual bool transmit(const uint8_t* data, size_t size, std::function<void()> completionCallback) = 0;
        virtual void setReceiveCallback(std::function<void(const uint8_t*, uint16_t)> callback) = 0;
    };
}

#endif /* BOLT_INTERFACE_H */
