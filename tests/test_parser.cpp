#include <gtest/gtest.h>
#include <cstring>
#include "parser.hpp"

// Helper: feed a byte array into the parser, return true if a frame was parsed
static bool feedBytes(bolt::FrameParser &parser, const uint8_t *data, size_t len, bolt::RawFrame &out)
{
    for (size_t i = 0; i < len; ++i)
    {
        if (parser.push(data[i], out))
            return true;
    }
    return false;
}

// Helper: build a valid frame into a buffer using build_frame from utils.h
static size_t makeFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen, uint8_t *out, size_t outCap)
{
    return build_frame(type, payload, payloadLen, out, outCap);
}

// ============================================================
// FrameParser Tests
// ============================================================

TEST(FrameParserTest, ParsesPingFrame)
{
    // AA 01 00 2E3E 55
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_Ping, nullptr, 0, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, len, raw));
    EXPECT_EQ(raw.type, bolt::FT_Ping);
    EXPECT_EQ(raw.len, 0);
}

TEST(FrameParserTest, ParsesFrameWithPayload)
{
    // MotorMove: motor=1, pulse=1000 (0x03E8)
    uint8_t payload[] = {0x01, 0x03, 0xE8};
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_MotorMove, payload, 3, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, len, raw));
    EXPECT_EQ(raw.type, bolt::FT_MotorMove);
    EXPECT_EQ(raw.len, 3);
    EXPECT_EQ(raw.payload[0], 0x01);
    EXPECT_EQ(raw.payload[1], 0x03);
    EXPECT_EQ(raw.payload[2], 0xE8);
}

TEST(FrameParserTest, RejectsInvalidCRC)
{
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_Ping, nullptr, 0, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    // Corrupt the CRC (second-to-last byte before EOF)
    buf[len - 2] ^= 0xFF;

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    EXPECT_FALSE(feedBytes(parser, buf, len, raw));
}

TEST(FrameParserTest, RejectsInvalidEOF)
{
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_Ping, nullptr, 0, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    // Corrupt EOF marker
    buf[len - 1] = 0x00;

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    EXPECT_FALSE(feedBytes(parser, buf, len, raw));
}

TEST(FrameParserTest, RejectsOversizedPayload)
{
    // Manually craft a frame with LEN > MAX_PAYLOAD
    uint8_t buf[] = {0xAA, 0x01, MAX_PAYLOAD + 1};

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    EXPECT_FALSE(feedBytes(parser, buf, sizeof(buf), raw));
}

TEST(FrameParserTest, IgnoresGarbageBeforeSOF)
{
    uint8_t frame[64];
    size_t frameLen = makeFrame(bolt::FT_Ping, nullptr, 0, frame, sizeof(frame));
    ASSERT_GT(frameLen, 0u);

    // Prepend garbage bytes
    uint8_t buf[128];
    buf[0] = 0x12;
    buf[1] = 0x34;
    buf[2] = 0x56;
    memcpy(buf + 3, frame, frameLen);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, 3 + frameLen, raw));
    EXPECT_EQ(raw.type, bolt::FT_Ping);
}

TEST(FrameParserTest, ParsesConsecutiveFrames)
{
    uint8_t ping[64], motor[64];
    size_t pingLen = makeFrame(bolt::FT_Ping, nullptr, 0, ping, sizeof(ping));
    uint8_t payload[] = {0x02, 0x01, 0xF4};
    size_t motorLen = makeFrame(bolt::FT_MotorMove, payload, 3, motor, sizeof(motor));

    uint8_t buf[128];
    memcpy(buf, ping, pingLen);
    memcpy(buf + pingLen, motor, motorLen);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};

    ASSERT_TRUE(feedBytes(parser, buf, pingLen, raw));
    EXPECT_EQ(raw.type, bolt::FT_Ping);

    ASSERT_TRUE(feedBytes(parser, buf + pingLen, motorLen, raw));
    EXPECT_EQ(raw.type, bolt::FT_MotorMove);
    EXPECT_EQ(raw.payload[0], 0x02);
}

TEST(FrameParserTest, ResyncsAfterBadFrameThenParsesGood)
{
    // Bad frame (corrupted CRC)
    uint8_t bad[64];
    size_t badLen = makeFrame(bolt::FT_Ping, nullptr, 0, bad, sizeof(bad));
    bad[badLen - 2] ^= 0xFF;

    // Good frame
    uint8_t good[64];
    size_t goodLen = makeFrame(bolt::FT_Ping, nullptr, 0, good, sizeof(good));

    uint8_t buf[128];
    memcpy(buf, bad, badLen);
    memcpy(buf + badLen, good, goodLen);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, badLen + goodLen, raw));
    EXPECT_EQ(raw.type, bolt::FT_Ping);
}

TEST(FrameParserTest, ZeroLenPayloadSkipsPayloadState)
{
    // EncoderGetValues has 0-length payload
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_EncoderGetValues, nullptr, 0, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, len, raw));
    EXPECT_EQ(raw.type, bolt::FT_EncoderGetValues);
    EXPECT_EQ(raw.len, 0);
}

// ============================================================
// FrameDecoder Tests
// ============================================================

TEST(FrameDecoderTest, DecodesPing)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_Ping;
    rf.len = 0;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->type, bolt::FT_Ping);
}

TEST(FrameDecoderTest, RejectsPingWithPayload)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_Ping;
    rf.len = 1;

    bolt::FrameDecoder decoder;
    EXPECT_EQ(decoder.decode(rf), nullptr);
}

TEST(FrameDecoderTest, DecodesMotorSpeed)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_MotorMove;
    rf.len = 3;
    rf.payload[0] = 2;          // motor_id
    rf.payload[1] = 0x03;       // pulse high byte
    rf.payload[2] = 0xE8;       // pulse low byte (1000)

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *msf = static_cast<const bolt::MotorSpeedFrame *>(f);
    EXPECT_EQ(msf->motor, 2);
    EXPECT_EQ(msf->pulse, 1000);
}

TEST(FrameDecoderTest, RejectsMotorSpeedWrongLen)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_MotorMove;
    rf.len = 2;

    bolt::FrameDecoder decoder;
    EXPECT_EQ(decoder.decode(rf), nullptr);
}

TEST(FrameDecoderTest, DecodesMotorStop)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_MotorStop;
    rf.len = 2;
    rf.payload[0] = 3;
    rf.payload[1] = 1;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *msf = static_cast<const bolt::MotorStopFrame *>(f);
    EXPECT_EQ(msf->motor, 3);
    EXPECT_EQ(msf->brake, 1);
}

TEST(FrameDecoderTest, DecodesPwmServo)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_ServoMove;
    rf.len = 2;
    rf.payload[0] = 1;
    rf.payload[1] = 90;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *psf = static_cast<const bolt::PwmServoFrame *>(f);
    EXPECT_EQ(psf->servo, 1);
    EXPECT_EQ(psf->angle, 90);
}

TEST(FrameDecoderTest, DecodesUartServo)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_UartServoMove;
    rf.len = 5;
    rf.payload[0] = 1;           // servo_id
    rf.payload[1] = 0x07;        // pulse high (2000)
    rf.payload[2] = 0xD0;        // pulse low
    rf.payload[3] = 0x01;        // time high (500)
    rf.payload[4] = 0xF4;        // time low

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *usf = static_cast<const bolt::UartServoFrame *>(f);
    EXPECT_EQ(usf->servo, 1);
    EXPECT_EQ(usf->pulse, 2000);
    EXPECT_EQ(usf->time, 500);
}

TEST(FrameDecoderTest, DecodesUartServoGetAngle)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_UartServoGetAngle;
    rf.len = 1;
    rf.payload[0] = 5;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *usf = static_cast<const bolt::UartServoGetAngleFrame *>(f);
    EXPECT_EQ(usf->servo, 5);
}

TEST(FrameDecoderTest, DecodesEncoderGetValues)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_EncoderGetValues;
    rf.len = 0;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->type, bolt::FT_EncoderGetValues);
}

TEST(FrameDecoderTest, DecodesImuGetValues)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_ImuGetValues;
    rf.len = 0;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->type, bolt::FT_ImuGetValues);
}

TEST(FrameDecoderTest, DecodesPidMotorSetRpm)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_PidMotorSetRpm;
    rf.len = 5;
    rf.payload[0] = 2;
    float rpm = 120.5f;
    memcpy(&rf.payload[1], &rpm, sizeof(float));

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *pf = static_cast<const bolt::PidMotorSetRpmFrame *>(f);
    EXPECT_EQ(pf->motor, 2);
    EXPECT_FLOAT_EQ(pf->rpm, 120.5f);
}

TEST(FrameDecoderTest, DecodesPidMotorStop)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_PidMotorStop;
    rf.len = 2;
    rf.payload[0] = 4;
    rf.payload[1] = 1;

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *pf = static_cast<const bolt::PidMotorStopFrame *>(f);
    EXPECT_EQ(pf->motor, 4);
    EXPECT_EQ(pf->brake, 1);
}

TEST(FrameDecoderTest, DecodesPidSetGains)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_PidSetGains;
    rf.len = 13;
    rf.payload[0] = 1;
    float kp = 1.5f, ki = 0.2f, kd = 0.05f;
    memcpy(&rf.payload[1], &kp, sizeof(float));
    memcpy(&rf.payload[5], &ki, sizeof(float));
    memcpy(&rf.payload[9], &kd, sizeof(float));

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(rf);
    ASSERT_NE(f, nullptr);

    auto *pf = static_cast<const bolt::PidSetGainsFrame *>(f);
    EXPECT_EQ(pf->motor, 1);
    EXPECT_FLOAT_EQ(pf->kp, 1.5f);
    EXPECT_FLOAT_EQ(pf->ki, 0.2f);
    EXPECT_FLOAT_EQ(pf->kd, 0.05f);
}

TEST(FrameDecoderTest, RejectsUnknownType)
{
    bolt::RawFrame rf{};
    rf.type = 0xFF;
    rf.len = 0;

    bolt::FrameDecoder decoder;
    EXPECT_EQ(decoder.decode(rf), nullptr);
}

TEST(FrameDecoderTest, RejectsPidSetGainsWrongLen)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_PidSetGains;
    rf.len = 10;

    bolt::FrameDecoder decoder;
    EXPECT_EQ(decoder.decode(rf), nullptr);
}

// ============================================================
// End-to-end: FrameParser + FrameDecoder
// ============================================================

TEST(ParserEndToEnd, ParseAndDecodeMotorSpeed)
{
    uint8_t payload[] = {0x03, 0x01, 0xF4}; // motor 3, pulse 500
    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_MotorMove, payload, 3, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, len, raw));

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(raw);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->type, bolt::FT_MotorMove);

    auto *msf = static_cast<const bolt::MotorSpeedFrame *>(f);
    EXPECT_EQ(msf->motor, 3);
    EXPECT_EQ(msf->pulse, 500);
}

TEST(ParserEndToEnd, ParseAndDecodePidSetGains)
{
    float kp = 2.0f, ki = 0.5f, kd = 0.1f;
    uint8_t payload[13];
    payload[0] = 1;
    memcpy(&payload[1], &kp, sizeof(float));
    memcpy(&payload[5], &ki, sizeof(float));
    memcpy(&payload[9], &kd, sizeof(float));

    uint8_t buf[64];
    size_t len = makeFrame(bolt::FT_PidSetGains, payload, 13, buf, sizeof(buf));
    ASSERT_GT(len, 0u);

    bolt::FrameParser parser;
    bolt::RawFrame raw{};
    ASSERT_TRUE(feedBytes(parser, buf, len, raw));

    bolt::FrameDecoder decoder;
    const bolt::Frame *f = decoder.decode(raw);
    ASSERT_NE(f, nullptr);

    auto *pf = static_cast<const bolt::PidSetGainsFrame *>(f);
    EXPECT_EQ(pf->motor, 1);
    EXPECT_FLOAT_EQ(pf->kp, 2.0f);
    EXPECT_FLOAT_EQ(pf->ki, 0.5f);
    EXPECT_FLOAT_EQ(pf->kd, 0.1f);
}
