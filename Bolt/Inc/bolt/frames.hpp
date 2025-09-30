#ifndef BOLT_FRAMES_HPP
#define BOLT_FRAMES_HPP

#include <cstdint>
#include <array>

#include "definitions.hpp"

namespace bolt
{
    enum FrameType : uint8_t
    {
        FT_Ping = 0x01,
        FT_MotorMove = 0x02,
        FT_MotorStop = 0x03,
        FT_ServoMove = 0x04,
        FT_UartServoMove = 0x05,
        FT_UartServoGetAngle = 0x06
    };

    struct PingFrame;
    struct MotorMoveFrame;
    struct MotorStopFrame;
    struct PwmServoFrame;
    struct UartServoFrame;
    struct UartServoGetAngleFrame;

    struct FrameVisitor
    {
        virtual ~FrameVisitor() {}
        virtual void visit(const PingFrame &f) = 0;
        virtual void visit(const MotorMoveFrame &f) = 0;
        virtual void visit(const MotorStopFrame &f) = 0;
        virtual void visit(const PwmServoFrame &f) = 0;
        virtual void visit(const UartServoFrame &f) = 0;
        virtual void visit(const UartServoGetAngleFrame &f) = 0;
    };

    struct Frame
    {
        const FrameType type;
        explicit Frame(FrameType t) : type(t) {}
        virtual ~Frame() {}
        virtual void accept(FrameVisitor &v) const = 0;
    };

    // Concrete Frames
    struct PingFrame : public Frame
    {
        PingFrame() : Frame(FT_Ping) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct MotorMoveFrame : public Frame
    {
        uint8_t motor;
        int16_t pulse;

        MotorMoveFrame() : Frame(FT_MotorMove), motor(0), pulse(0) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct MotorStopFrame : public Frame
    {
        uint8_t motor;
        uint8_t brake;

        MotorStopFrame() : Frame(FT_MotorStop), brake(0) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct PwmServoFrame : public Frame
    {
        uint8_t servo;
        uint8_t angle;

        PwmServoFrame() : Frame(FT_ServoMove), servo(0), angle(0) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct UartServoFrame : public Frame
    {
        uint8_t servo;
        uint16_t pulse;
        uint16_t time;

        UartServoFrame() : Frame(FT_UartServoMove), servo(0), pulse(0), time(0) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct UartServoGetAngleFrame : public Frame
    {
        uint8_t servo;

        UartServoGetAngleFrame() : Frame(FT_UartServoGetAngle), servo(0) {}
        void accept(FrameVisitor &v) const { v.visit(*this); }
    };

    struct RawFrame
    {
        uint8_t type;
        uint8_t len;
        uint8_t payload[MAX_PAYLOAD];
        uint16_t crc; // received CRC
    };
}

#endif /* BOLT_FRAMES_HPP */
