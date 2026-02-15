#ifndef BOLT_DEFINITIONS_HPP
#define BOLT_DEFINITIONS_HPP

#define BUFF_SIZE 38

enum : uint8_t
{
    SOF = 0xAA,
    EOF_ = 0x55,
    MAX_PAYLOAD = 32
};

enum : uint8_t
{
    PONG = 0x01,
    RPMS = 0x02,
    ANGLE = 0x03,
    IMU = 0x04,
};

#endif /* BOLT_DEFINITIONS_HPP */
