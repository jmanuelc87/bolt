#ifndef BOLT_INTERFACE_HPP
#define BOLT_INTERFACE_HPP

#include <functional>
#include <cstdint>

typedef enum
{
    MOTOR_ID_M1 = 1,
    MOTOR_ID_M2,
    MOTOR_ID_M3,
    MOTOR_ID_M4,
    MAX_MOTOR
} Motor_ID;

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

    class PWMTimer
    {
    public:
        virtual bool setPulse(int16_t pulse, Motor_ID motor) = 0;
    };
}

#endif /* BOLT_INTERFACE_HPP */
