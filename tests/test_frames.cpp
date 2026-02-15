#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include "frames.hpp"

namespace
{
    enum VisitedFrame
    {
        NONE,
        PING,
        MOTOR_SPEED,
        MOTOR_STOP,
        PWM_SERVO,
        UART_SERVO,
        UART_SERVO_GET_ANGLE,
        ENCODER_GET_VALUES,
        IMU_GET_VALUES,
        PID_MOTOR_SET_RPM,
        PID_MOTOR_STOP,
        PID_SET_GAINS
    };

    // MockVisitor tracks which visit() was called last.
    // Because `called` starts as NONE, any test that checks a specific value
    // also implicitly verifies no earlier visit() fired (a single accept() call
    // changes NONE to exactly one enum value).
    struct MockVisitor : public bolt::FrameVisitor
    {
        VisitedFrame called = NONE;
        int total_calls = 0;

        void visit(const bolt::PingFrame &) override
        {
            called = PING;
            ++total_calls;
        }
        void visit(const bolt::MotorSpeedFrame &) override
        {
            called = MOTOR_SPEED;
            ++total_calls;
        }
        void visit(const bolt::MotorStopFrame &) override
        {
            called = MOTOR_STOP;
            ++total_calls;
        }
        void visit(const bolt::PwmServoFrame &) override
        {
            called = PWM_SERVO;
            ++total_calls;
        }
        void visit(const bolt::UartServoFrame &) override
        {
            called = UART_SERVO;
            ++total_calls;
        }
        void visit(const bolt::UartServoGetAngleFrame &) override
        {
            called = UART_SERVO_GET_ANGLE;
            ++total_calls;
        }
        void visit(const bolt::EncoderGetValuesFrame &) override
        {
            called = ENCODER_GET_VALUES;
            ++total_calls;
        }
        void visit(const bolt::ImuGetValuesFrame &) override
        {
            called = IMU_GET_VALUES;
            ++total_calls;
        }
        void visit(const bolt::PidMotorSetRpmFrame &) override
        {
            called = PID_MOTOR_SET_RPM;
            ++total_calls;
        }
        void visit(const bolt::PidMotorStopFrame &) override
        {
            called = PID_MOTOR_STOP;
            ++total_calls;
        }
        void visit(const bolt::PidSetGainsFrame &) override
        {
            called = PID_SET_GAINS;
            ++total_calls;
        }
    };
} // anonymous namespace

// ============================================================
// FrameType enum — wire-protocol numeric values (0x01–0x0B)
// These must match the binary frame spec exactly.
// ============================================================

TEST(FrameTypeEnumTest, PingValue_Is_0x01)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_Ping), 0x01u);
}

TEST(FrameTypeEnumTest, MotorMoveValue_Is_0x02)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_MotorMove), 0x02u);
}

TEST(FrameTypeEnumTest, MotorStopValue_Is_0x03)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_MotorStop), 0x03u);
}

TEST(FrameTypeEnumTest, ServoMoveValue_Is_0x04)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_ServoMove), 0x04u);
}

TEST(FrameTypeEnumTest, UartServoMoveValue_Is_0x05)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_UartServoMove), 0x05u);
}

TEST(FrameTypeEnumTest, UartServoGetAngleValue_Is_0x06)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_UartServoGetAngle), 0x06u);
}

TEST(FrameTypeEnumTest, EncoderGetValuesValue_Is_0x07)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_EncoderGetValues), 0x07u);
}

TEST(FrameTypeEnumTest, ImuGetValuesValue_Is_0x08)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_ImuGetValues), 0x08u);
}

TEST(FrameTypeEnumTest, PidMotorSetRpmValue_Is_0x09)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_PidMotorSetRpm), 0x09u);
}

TEST(FrameTypeEnumTest, PidMotorStopValue_Is_0x0A)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_PidMotorStop), 0x0Au);
}

TEST(FrameTypeEnumTest, PidSetGainsValue_Is_0x0B)
{
    EXPECT_EQ(static_cast<uint8_t>(bolt::FT_PidSetGains), 0x0Bu);
}

TEST(FrameTypeEnumTest, EnumUnderlyingTypeIsUint8)
{
    // Verify the enum's underlying type is uint8_t (not int), matching
    // the protocol requirement that the TYPE byte is exactly one byte.
    static_assert(
        std::is_same<std::underlying_type<bolt::FrameType>::type, uint8_t>::value,
        "FrameType must have underlying type uint8_t");
    SUCCEED();
}

// ============================================================
// RawFrame struct
// ============================================================

TEST(RawFrameTest, PayloadArraySize_EqualsMAX_PAYLOAD)
{
    bolt::RawFrame rf{};
    // MAX_PAYLOAD is 32; the payload array must be exactly that size.
    EXPECT_EQ(sizeof(rf.payload), static_cast<size_t>(MAX_PAYLOAD));
}

TEST(RawFrameTest, TypeFieldWriteAndRead)
{
    bolt::RawFrame rf{};
    rf.type = bolt::FT_MotorMove;
    EXPECT_EQ(rf.type, bolt::FT_MotorMove);
}

TEST(RawFrameTest, LenFieldWriteAndRead)
{
    bolt::RawFrame rf{};
    rf.len = 5u;
    EXPECT_EQ(rf.len, 5u);
}

TEST(RawFrameTest, CrcFieldWriteAndRead)
{
    bolt::RawFrame rf{};
    rf.crc = 0xABCDu;
    EXPECT_EQ(rf.crc, 0xABCDu);
}

TEST(RawFrameTest, PayloadBytesWriteAndRead)
{
    bolt::RawFrame rf{};
    for (uint8_t i = 0; i < MAX_PAYLOAD; ++i)
        rf.payload[i] = i;

    for (uint8_t i = 0; i < MAX_PAYLOAD; ++i)
        EXPECT_EQ(rf.payload[i], i) << "Payload byte " << static_cast<int>(i) << " mismatch";
}

TEST(RawFrameTest, ZeroInitialisedStructHasZeroFields)
{
    bolt::RawFrame rf{};
    EXPECT_EQ(rf.type, 0u);
    EXPECT_EQ(rf.len, 0u);
    EXPECT_EQ(rf.crc, 0u);
    for (size_t i = 0; i < MAX_PAYLOAD; ++i)
        EXPECT_EQ(rf.payload[i], 0u) << "payload[" << i << "] should be zero";
}

TEST(RawFrameTest, PayloadMaxIndexWriteAndRead)
{
    bolt::RawFrame rf{};
    rf.payload[MAX_PAYLOAD - 1] = 0xBEu;
    EXPECT_EQ(rf.payload[MAX_PAYLOAD - 1], 0xBEu);
}

TEST(RawFrameTest, CrcMaxValue_0xFFFF)
{
    bolt::RawFrame rf{};
    rf.crc = 0xFFFFu;
    EXPECT_EQ(rf.crc, 0xFFFFu);
}

// ============================================================
// PingFrame
// ============================================================

TEST(PingFrameTest, HasCorrectType)
{
    bolt::PingFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_Ping);
}

TEST(PingFrameTest, AcceptCallsVisitPing)
{
    bolt::PingFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, PING);
}

TEST(PingFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::PingFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1) << "accept() must call exactly one visit() overload";
}

TEST(PingFrameTest, TypeAccessibleViaBasePointer)
{
    bolt::PingFrame frame;
    const bolt::Frame *base = &frame;
    EXPECT_EQ(base->type, bolt::FT_Ping);
}

TEST(PingFrameTest, AcceptViaBasePointerDispatchesCorrectly)
{
    bolt::PingFrame frame;
    bolt::Frame *base = &frame;
    MockVisitor visitor;
    base->accept(visitor);
    EXPECT_EQ(visitor.called, PING);
}

// ============================================================
// MotorSpeedFrame
// ============================================================

TEST(MotorSpeedFrameTest, HasCorrectType)
{
    bolt::MotorSpeedFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_MotorMove);
}

TEST(MotorSpeedFrameTest, AcceptCallsCorrectVisit)
{
    bolt::MotorSpeedFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, MOTOR_SPEED);
}

TEST(MotorSpeedFrameTest, DefaultFieldValues)
{
    bolt::MotorSpeedFrame frame;
    EXPECT_EQ(frame.motor, 0u);
    EXPECT_EQ(frame.pulse, 0);
}

TEST(MotorSpeedFrameTest, MotorFieldWriteAndRead)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 4u;
    EXPECT_EQ(frame.motor, 4u);
}

TEST(MotorSpeedFrameTest, PositivePulseFieldWriteAndRead)
{
    bolt::MotorSpeedFrame frame;
    frame.pulse = 2000;
    EXPECT_EQ(frame.pulse, 2000);
}

TEST(MotorSpeedFrameTest, NegativePulseFieldPreservesSign)
{
    // int16_t can hold negative values; reverse direction uses negative pulse.
    bolt::MotorSpeedFrame frame;
    frame.pulse = static_cast<int16_t>(-1600);
    EXPECT_EQ(frame.pulse, static_cast<int16_t>(-1600));
}

TEST(MotorSpeedFrameTest, MaxPositivePulseValue)
{
    bolt::MotorSpeedFrame frame;
    frame.pulse = INT16_MAX;
    EXPECT_EQ(frame.pulse, INT16_MAX);
}

TEST(MotorSpeedFrameTest, MaxNegativePulseValue)
{
    bolt::MotorSpeedFrame frame;
    frame.pulse = INT16_MIN;
    EXPECT_EQ(frame.pulse, INT16_MIN);
}

TEST(MotorSpeedFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::MotorSpeedFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

TEST(MotorSpeedFrameTest, TypeAccessibleViaBasePointer)
{
    bolt::MotorSpeedFrame frame;
    const bolt::Frame *base = &frame;
    EXPECT_EQ(base->type, bolt::FT_MotorMove);
}

// ============================================================
// MotorStopFrame
// ============================================================

TEST(MotorStopFrameTest, HasCorrectType)
{
    bolt::MotorStopFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_MotorStop);
}

TEST(MotorStopFrameTest, AcceptCallsCorrectVisit)
{
    bolt::MotorStopFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, MOTOR_STOP);
}

TEST(MotorStopFrameTest, DefaultBrakeValue)
{
    bolt::MotorStopFrame frame;
    EXPECT_EQ(frame.brake, 0u);
}

TEST(MotorStopFrameTest, MotorFieldWriteAndRead)
{
    bolt::MotorStopFrame frame;
    frame.motor = 3u;
    EXPECT_EQ(frame.motor, 3u);
}

TEST(MotorStopFrameTest, BrakeFieldWriteAndRead)
{
    bolt::MotorStopFrame frame;
    frame.brake = 1u;
    EXPECT_EQ(frame.brake, 1u);
}

TEST(MotorStopFrameTest, BrakeMaxByteValue)
{
    bolt::MotorStopFrame frame;
    frame.brake = 0xFFu;
    EXPECT_EQ(frame.brake, 0xFFu);
}

TEST(MotorStopFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::MotorStopFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// PwmServoFrame
// ============================================================

TEST(PwmServoFrameTest, HasCorrectType)
{
    bolt::PwmServoFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_ServoMove);
}

TEST(PwmServoFrameTest, AcceptCallsCorrectVisit)
{
    bolt::PwmServoFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, PWM_SERVO);
}

TEST(PwmServoFrameTest, DefaultFieldValues)
{
    bolt::PwmServoFrame frame;
    EXPECT_EQ(frame.servo, 0u);
    EXPECT_EQ(frame.angle, 0u);
}

TEST(PwmServoFrameTest, ServoFieldWriteAndRead)
{
    bolt::PwmServoFrame frame;
    frame.servo = 3u;
    EXPECT_EQ(frame.servo, 3u);
}

TEST(PwmServoFrameTest, AngleFieldWriteAndRead)
{
    bolt::PwmServoFrame frame;
    frame.angle = 90u;
    EXPECT_EQ(frame.angle, 90u);
}

TEST(PwmServoFrameTest, AngleMaxByteValue)
{
    bolt::PwmServoFrame frame;
    frame.angle = 0xFFu;
    EXPECT_EQ(frame.angle, 0xFFu);
}

TEST(PwmServoFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::PwmServoFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// UartServoFrame
// ============================================================

TEST(UartServoFrameTest, HasCorrectType)
{
    bolt::UartServoFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_UartServoMove);
}

TEST(UartServoFrameTest, AcceptCallsCorrectVisit)
{
    bolt::UartServoFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, UART_SERVO);
}

TEST(UartServoFrameTest, DefaultFieldValues)
{
    bolt::UartServoFrame frame;
    EXPECT_EQ(frame.servo, 0u);
    EXPECT_EQ(frame.pulse, 0u);
    EXPECT_EQ(frame.time, 0u);
}

TEST(UartServoFrameTest, ServoFieldWriteAndRead)
{
    bolt::UartServoFrame frame;
    frame.servo = 6u;
    EXPECT_EQ(frame.servo, 6u);
}

TEST(UartServoFrameTest, PulseMinBoundaryValue)
{
    // Minimum valid servo pulse per UART servo spec is 96.
    bolt::UartServoFrame frame;
    frame.pulse = 96u;
    EXPECT_EQ(frame.pulse, 96u);
}

TEST(UartServoFrameTest, PulseMaxBoundaryValue)
{
    // Maximum valid servo pulse per UART servo spec is 4000.
    bolt::UartServoFrame frame;
    frame.pulse = 4000u;
    EXPECT_EQ(frame.pulse, 4000u);
}

TEST(UartServoFrameTest, PulseUint16MaxValue)
{
    // Field is uint16_t — it must hold 0xFFFF without truncation.
    bolt::UartServoFrame frame;
    frame.pulse = 0xFFFFu;
    EXPECT_EQ(frame.pulse, 0xFFFFu);
}

TEST(UartServoFrameTest, TimeFieldWriteAndRead)
{
    bolt::UartServoFrame frame;
    frame.time = 1000u;
    EXPECT_EQ(frame.time, 1000u);
}

TEST(UartServoFrameTest, TimeUint16MaxValue)
{
    bolt::UartServoFrame frame;
    frame.time = 0xFFFFu;
    EXPECT_EQ(frame.time, 0xFFFFu);
}

TEST(UartServoFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::UartServoFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// UartServoGetAngleFrame
// ============================================================

TEST(UartServoGetAngleFrameTest, HasCorrectType)
{
    bolt::UartServoGetAngleFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_UartServoGetAngle);
}

TEST(UartServoGetAngleFrameTest, AcceptCallsCorrectVisit)
{
    bolt::UartServoGetAngleFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, UART_SERVO_GET_ANGLE);
}

TEST(UartServoGetAngleFrameTest, DefaultServoValue)
{
    bolt::UartServoGetAngleFrame frame;
    EXPECT_EQ(frame.servo, 0u);
}

TEST(UartServoGetAngleFrameTest, ServoFieldWriteAndRead)
{
    bolt::UartServoGetAngleFrame frame;
    frame.servo = 5u;
    EXPECT_EQ(frame.servo, 5u);
}

TEST(UartServoGetAngleFrameTest, ServoMaxByteValue)
{
    bolt::UartServoGetAngleFrame frame;
    frame.servo = 0xFFu;
    EXPECT_EQ(frame.servo, 0xFFu);
}

TEST(UartServoGetAngleFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::UartServoGetAngleFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// EncoderGetValuesFrame
// ============================================================

TEST(EncoderGetValuesFrameTest, HasCorrectType)
{
    bolt::EncoderGetValuesFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_EncoderGetValues);
}

TEST(EncoderGetValuesFrameTest, AcceptCallsCorrectVisit)
{
    bolt::EncoderGetValuesFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, ENCODER_GET_VALUES);
}

TEST(EncoderGetValuesFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::EncoderGetValuesFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

TEST(EncoderGetValuesFrameTest, TypeAccessibleViaBasePointer)
{
    bolt::EncoderGetValuesFrame frame;
    const bolt::Frame *base = &frame;
    EXPECT_EQ(base->type, bolt::FT_EncoderGetValues);
}

// ============================================================
// ImuGetValuesFrame
// ============================================================

TEST(ImuGetValuesFrameTest, HasCorrectType)
{
    bolt::ImuGetValuesFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_ImuGetValues);
}

TEST(ImuGetValuesFrameTest, AcceptCallsCorrectVisit)
{
    bolt::ImuGetValuesFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, IMU_GET_VALUES);
}

TEST(ImuGetValuesFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::ImuGetValuesFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

TEST(ImuGetValuesFrameTest, TypeAccessibleViaBasePointer)
{
    bolt::ImuGetValuesFrame frame;
    const bolt::Frame *base = &frame;
    EXPECT_EQ(base->type, bolt::FT_ImuGetValues);
}

// ============================================================
// PidMotorSetRpmFrame
// ============================================================

TEST(PidMotorSetRpmFrameTest, HasCorrectType)
{
    bolt::PidMotorSetRpmFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_PidMotorSetRpm);
}

TEST(PidMotorSetRpmFrameTest, AcceptCallsCorrectVisit)
{
    bolt::PidMotorSetRpmFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, PID_MOTOR_SET_RPM);
}

TEST(PidMotorSetRpmFrameTest, DefaultFieldValues)
{
    bolt::PidMotorSetRpmFrame frame;
    EXPECT_EQ(frame.motor, 0u);
    EXPECT_FLOAT_EQ(frame.rpm, 0.0f);
}

TEST(PidMotorSetRpmFrameTest, MotorFieldWriteAndRead)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 4u;
    EXPECT_EQ(frame.motor, 4u);
}

TEST(PidMotorSetRpmFrameTest, PositiveRpmFieldWriteAndRead)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.rpm = 150.5f;
    EXPECT_FLOAT_EQ(frame.rpm, 150.5f);
}

TEST(PidMotorSetRpmFrameTest, NegativeRpmFieldPreservesSign)
{
    // Negative RPM represents reverse rotation direction.
    bolt::PidMotorSetRpmFrame frame;
    frame.rpm = -120.0f;
    EXPECT_FLOAT_EQ(frame.rpm, -120.0f);
}

TEST(PidMotorSetRpmFrameTest, ZeroRpmField)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.rpm = 0.0f;
    EXPECT_FLOAT_EQ(frame.rpm, 0.0f);
}

TEST(PidMotorSetRpmFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::PidMotorSetRpmFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// PidMotorStopFrame
// ============================================================

TEST(PidMotorStopFrameTest, HasCorrectType)
{
    bolt::PidMotorStopFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_PidMotorStop);
}

TEST(PidMotorStopFrameTest, AcceptCallsCorrectVisit)
{
    bolt::PidMotorStopFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, PID_MOTOR_STOP);
}

TEST(PidMotorStopFrameTest, DefaultFieldValues)
{
    bolt::PidMotorStopFrame frame;
    EXPECT_EQ(frame.motor, 0u);
    EXPECT_EQ(frame.brake, 0u);
}

TEST(PidMotorStopFrameTest, MotorFieldWriteAndRead)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 2u;
    EXPECT_EQ(frame.motor, 2u);
}

TEST(PidMotorStopFrameTest, BrakeOneValue)
{
    bolt::PidMotorStopFrame frame;
    frame.brake = 1u;
    EXPECT_EQ(frame.brake, 1u);
}

TEST(PidMotorStopFrameTest, BrakeMaxByteValue)
{
    bolt::PidMotorStopFrame frame;
    frame.brake = 0xFFu;
    EXPECT_EQ(frame.brake, 0xFFu);
}

TEST(PidMotorStopFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::PidMotorStopFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// PidSetGainsFrame
// ============================================================

TEST(PidSetGainsFrameTest, HasCorrectType)
{
    bolt::PidSetGainsFrame frame;
    EXPECT_EQ(frame.type, bolt::FT_PidSetGains);
}

TEST(PidSetGainsFrameTest, AcceptCallsCorrectVisit)
{
    bolt::PidSetGainsFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.called, PID_SET_GAINS);
}

TEST(PidSetGainsFrameTest, DefaultFieldValues)
{
    bolt::PidSetGainsFrame frame;
    EXPECT_EQ(frame.motor, 0u);
    EXPECT_FLOAT_EQ(frame.kp, 0.0f);
    EXPECT_FLOAT_EQ(frame.ki, 0.0f);
    EXPECT_FLOAT_EQ(frame.kd, 0.0f);
}

TEST(PidSetGainsFrameTest, MotorFieldWriteAndRead)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 3u;
    EXPECT_EQ(frame.motor, 3u);
}

TEST(PidSetGainsFrameTest, KpFieldWriteAndRead)
{
    bolt::PidSetGainsFrame frame;
    frame.kp = 1.5f;
    EXPECT_FLOAT_EQ(frame.kp, 1.5f);
}

TEST(PidSetGainsFrameTest, KiFieldWriteAndRead)
{
    bolt::PidSetGainsFrame frame;
    frame.ki = 0.2f;
    EXPECT_FLOAT_EQ(frame.ki, 0.2f);
}

TEST(PidSetGainsFrameTest, KdFieldWriteAndRead)
{
    bolt::PidSetGainsFrame frame;
    frame.kd = 0.05f;
    EXPECT_FLOAT_EQ(frame.kd, 0.05f);
}

TEST(PidSetGainsFrameTest, AllGainsSetIndependently)
{
    // Verify that setting kp, ki, kd does not alias or corrupt one another.
    bolt::PidSetGainsFrame frame;
    frame.kp = 2.0f;
    frame.ki = 0.5f;
    frame.kd = 0.1f;
    EXPECT_FLOAT_EQ(frame.kp, 2.0f);
    EXPECT_FLOAT_EQ(frame.ki, 0.5f);
    EXPECT_FLOAT_EQ(frame.kd, 0.1f);
}

TEST(PidSetGainsFrameTest, NegativeGainsAreHeld)
{
    // The struct imposes no range validation — it must store any float.
    bolt::PidSetGainsFrame frame;
    frame.kp = -1.0f;
    frame.ki = -0.5f;
    frame.kd = -0.05f;
    EXPECT_FLOAT_EQ(frame.kp, -1.0f);
    EXPECT_FLOAT_EQ(frame.ki, -0.5f);
    EXPECT_FLOAT_EQ(frame.kd, -0.05f);
}

TEST(PidSetGainsFrameTest, AcceptDispatchesToExactlyOneVisitMethod)
{
    bolt::PidSetGainsFrame frame;
    MockVisitor visitor;
    frame.accept(visitor);
    EXPECT_EQ(visitor.total_calls, 1);
}

// ============================================================
// Visitor exclusivity — each frame type triggers only its own visit()
// ============================================================
//
// The MockVisitor.total_calls counter makes these assertions redundant with
// the AcceptDispatchesToExactlyOneVisitMethod tests above, but the explicit
// NONE-then-X pattern documents the contract at the test level.

TEST(VisitorExclusivityTest, PingFrame_DoesNotTriggerOtherVisits)
{
    bolt::PingFrame frame;
    MockVisitor v;
    EXPECT_EQ(v.called, NONE) << "Precondition: no visit() called yet";
    frame.accept(v);
    EXPECT_EQ(v.called, PING);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, MotorSpeedFrame_DoesNotTriggerOtherVisits)
{
    bolt::MotorSpeedFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, MOTOR_SPEED);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, MotorStopFrame_DoesNotTriggerOtherVisits)
{
    bolt::MotorStopFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, MOTOR_STOP);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, PwmServoFrame_DoesNotTriggerOtherVisits)
{
    bolt::PwmServoFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, PWM_SERVO);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, UartServoFrame_DoesNotTriggerOtherVisits)
{
    bolt::UartServoFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, UART_SERVO);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, UartServoGetAngleFrame_DoesNotTriggerOtherVisits)
{
    bolt::UartServoGetAngleFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, UART_SERVO_GET_ANGLE);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, EncoderGetValuesFrame_DoesNotTriggerOtherVisits)
{
    bolt::EncoderGetValuesFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, ENCODER_GET_VALUES);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, ImuGetValuesFrame_DoesNotTriggerOtherVisits)
{
    bolt::ImuGetValuesFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, IMU_GET_VALUES);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, PidMotorSetRpmFrame_DoesNotTriggerOtherVisits)
{
    bolt::PidMotorSetRpmFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, PID_MOTOR_SET_RPM);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, PidMotorStopFrame_DoesNotTriggerOtherVisits)
{
    bolt::PidMotorStopFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, PID_MOTOR_STOP);
    EXPECT_EQ(v.total_calls, 1);
}

TEST(VisitorExclusivityTest, PidSetGainsFrame_DoesNotTriggerOtherVisits)
{
    bolt::PidSetGainsFrame frame;
    MockVisitor v;
    frame.accept(v);
    EXPECT_EQ(v.called, PID_SET_GAINS);
    EXPECT_EQ(v.total_calls, 1);
}

// ============================================================
// Polymorphic dispatch via Frame base pointer
//
// Verifies that accept() is truly virtual: calling through a Frame*
// reaches the concrete frame's visit() overload.
// ============================================================

TEST(PolymorphicDispatchTest, BasePointerToPingFrame_DispatchesPing)
{
    bolt::PingFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, PING);
}

TEST(PolymorphicDispatchTest, BasePointerToMotorSpeedFrame_DispatchesMotorSpeed)
{
    bolt::MotorSpeedFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, MOTOR_SPEED);
}

TEST(PolymorphicDispatchTest, BasePointerToMotorStopFrame_DispatchesMotorStop)
{
    bolt::MotorStopFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, MOTOR_STOP);
}

TEST(PolymorphicDispatchTest, BasePointerToPwmServoFrame_DispatchesPwmServo)
{
    bolt::PwmServoFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, PWM_SERVO);
}

TEST(PolymorphicDispatchTest, BasePointerToUartServoFrame_DispatchesUartServo)
{
    bolt::UartServoFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, UART_SERVO);
}

TEST(PolymorphicDispatchTest, BasePointerToUartServoGetAngleFrame_DispatchesGetAngle)
{
    bolt::UartServoGetAngleFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, UART_SERVO_GET_ANGLE);
}

TEST(PolymorphicDispatchTest, BasePointerToEncoderGetValuesFrame_DispatchesEncoder)
{
    bolt::EncoderGetValuesFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, ENCODER_GET_VALUES);
}

TEST(PolymorphicDispatchTest, BasePointerToImuGetValuesFrame_DispatchesImu)
{
    bolt::ImuGetValuesFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, IMU_GET_VALUES);
}

TEST(PolymorphicDispatchTest, BasePointerToPidMotorSetRpmFrame_DispatchesPidSetRpm)
{
    bolt::PidMotorSetRpmFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, PID_MOTOR_SET_RPM);
}

TEST(PolymorphicDispatchTest, BasePointerToPidMotorStopFrame_DispatchesPidStop)
{
    bolt::PidMotorStopFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, PID_MOTOR_STOP);
}

TEST(PolymorphicDispatchTest, BasePointerToPidSetGainsFrame_DispatchesPidSetGains)
{
    bolt::PidSetGainsFrame concrete;
    bolt::Frame *base = &concrete;
    MockVisitor v;
    base->accept(v);
    EXPECT_EQ(v.called, PID_SET_GAINS);
}

// ============================================================
// Polymorphic deletion via FrameVisitor base pointer
//
// Verifies that FrameVisitor has a virtual destructor: deleting through a
// base pointer must not leak the derived object.  This is a link-time and
// runtime safety contract.
// ============================================================

TEST(FrameVisitorDestructorTest, DeletingThroughBasePointerIsWellDefined)
{
    // If FrameVisitor lacked a virtual destructor this would be UB; the test
    // verifies the class compiles and runs without sanitizer errors.
    bolt::FrameVisitor *v = new MockVisitor();
    ASSERT_NO_FATAL_FAILURE(delete v);
}

// ============================================================
// Frame type field is const — it cannot be reassigned after construction.
// (compile-time contract; verified by the static_assert below.)
// ============================================================

TEST(FrameTypeConstTest, TypeFieldIsConst)
{
    // Verify that Frame::type is a const member.  std::is_const on a member
    // type is checked via the struct's field access type.
    static_assert(
        std::is_const<decltype(bolt::PingFrame::type)>::value,
        "Frame::type must be const to prevent accidental mutation after construction");
    SUCCEED();
}

// ============================================================
// Sequential accept() calls on separate frames use independent visitors
// ============================================================

TEST(SequentialAcceptTest, TwoFramesTwoVisitors_EachCalledOnce)
{
    bolt::PingFrame ping;
    bolt::MotorSpeedFrame motor;

    MockVisitor v1, v2;
    ping.accept(v1);
    motor.accept(v2);

    EXPECT_EQ(v1.called, PING) << "v1 must only see the Ping frame";
    EXPECT_EQ(v2.called, MOTOR_SPEED) << "v2 must only see the MotorSpeed frame";
    EXPECT_EQ(v1.total_calls, 1);
    EXPECT_EQ(v2.total_calls, 1);
}

TEST(SequentialAcceptTest, SameVisitorCalledWithTwoFrames_TotalCallsIsTwo)
{
    bolt::PingFrame ping;
    bolt::MotorStopFrame stop;

    MockVisitor v;
    ping.accept(v);
    stop.accept(v);

    // After the second accept(), called is overwritten to MOTOR_STOP.
    EXPECT_EQ(v.called, MOTOR_STOP);
    EXPECT_EQ(v.total_calls, 2);
}
