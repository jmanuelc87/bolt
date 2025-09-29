#ifndef BOLT_PARSER_HPP
#define BOLT_PARSER_HPP

#include <cstdint>

#include "frames.hpp"
#include "utils.h"

using std::size_t;

namespace bolt
{
    class FrameParser
    {
    public:
        FrameParser() { reset(); }

        // Feed bytes one by one (e.g., from UART RX IRQ or DMA ring).
        // Returns true if a frame was parsed into outRaw.
        bool push(uint8_t byte, RawFrame &outRaw)
        {
            switch (state_)
            {
            case S_WAIT_SOF:
                if (byte == SOF)
                {
                    state_ = S_TYPE;
                }
                break;

            case S_TYPE:
                cur_.type = byte;
                state_ = S_LEN;
                break;

            case S_LEN:
                cur_.len = byte;
                if (cur_.len > MAX_PAYLOAD)
                {
                    reset();
                    break;
                }
                idx_ = 0;
                state_ = (cur_.len == 0) ? S_CRC_HI : S_PAYLOAD;
                break;

            case S_PAYLOAD:
                cur_.payload[idx_++] = byte;
                if (idx_ >= cur_.len)
                {
                    state_ = S_CRC_HI;
                }
                break;

            case S_CRC_HI:
                crc_hi_ = byte;
                state_ = S_CRC_LO;
                break;

            case S_CRC_LO:
                crc_lo_ = byte;
                cur_.crc = (uint16_t)((crc_hi_ << 8) | crc_lo_);
                state_ = S_EOF;
                break;

            case S_EOF:
                if (byte != EOF_)
                {
                    reset();
                    break;
                }
                // validate CRC over [TYPE][LEN][PAYLOAD]
                {
                    uint8_t tmp[2 + MAX_PAYLOAD];
                    tmp[0] = cur_.type;
                    tmp[1] = cur_.len;
                    for (uint8_t i = 0; i < cur_.len; ++i)
                        tmp[2 + i] = cur_.payload[i];
                    const uint16_t c = crc16_ccitt(tmp, (size_t)2 + cur_.len);
                    if (c == cur_.crc)
                    {
                        outRaw = cur_;
                        reset(); // ready for next
                        return true;
                    }
                    else
                    {
                        // CRC error; resync by looking for a new SOF next
                        reset();
                    }
                }
                break;
            }
            return false;
        }

    private:
        enum State
        {
            S_WAIT_SOF,
            S_TYPE,
            S_LEN,
            S_PAYLOAD,
            S_CRC_HI,
            S_CRC_LO,
            S_EOF
        } state_;
        RawFrame cur_;
        uint8_t idx_;
        uint8_t crc_hi_, crc_lo_;

        void reset()
        {
            state_ = S_WAIT_SOF;
            cur_.type = 0;
            cur_.len = 0;
            idx_ = 0;
            crc_hi_ = 0;
            crc_lo_ = 0;
        }
    };

    class FrameDecoder
    {
    public:
        // Decodes RawFrame into one of the statically-owned objects below.
        // Returns nullptr if type/length invalid.
        const Frame *decode(const RawFrame &rf)
        {
            switch (rf.type)
            {
            case FT_Ping:
                if (rf.len != 0)
                    return 0;
                return &ping_;

            case FT_MotorMove:
                if (rf.len != 3)
                    return 0;

                setmtr_.motor = rf.payload[0];
                setmtr_.pulse = u16be(&rf.payload[1]);

                return &setmtr_;

            case FT_MotorStop:
                if (rf.len != 2)
                    return 0;

                stop_.motor = rf.payload[0];
                stop_.brake = rf.payload[1];

                return &stop_;

            case FT_ServoMove:
                if (rf.len != 2)
                    return 0;

                sm_.servo = rf.payload[0];
                sm_.angle = rf.payload[1];

                return &sm_;
            }
            return 0;
        }

    private:
        PingFrame ping_;
        MotorMoveFrame setmtr_;
        MotorStopFrame stop_;
        PwmServoFrame sm_;
    };
}

#endif /* BOLT_PARSER_HPP */
