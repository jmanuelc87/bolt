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
        FT_SetMotor = 0x02
    };

    struct PingFrame;
    struct SetMotorFrame;

    struct FrameVisitor
    {
        virtual ~FrameVisitor() {}
        virtual void visit(const PingFrame &f) = 0;
        virtual void visit(const SetMotorFrame &f) = 0;
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

    struct SetMotorFrame : public Frame
    {
        uint8_t key;
        uint32_t value;

        SetMotorFrame() : Frame(FT_SetMotor), key(0), value(0) {}
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
