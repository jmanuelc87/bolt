#include <gtest/gtest.h>
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

    struct MockVisitor : public bolt::FrameVisitor
    {
        VisitedFrame called = NONE;

        void visit(const bolt::PingFrame &) override { called = PING; }
        void visit(const bolt::MotorSpeedFrame &) override { called = MOTOR_SPEED; }
        void visit(const bolt::MotorStopFrame &) override { called = MOTOR_STOP; }
        void visit(const bolt::PwmServoFrame &) override { called = PWM_SERVO; }
        void visit(const bolt::UartServoFrame &) override { called = UART_SERVO; }
        void visit(const bolt::UartServoGetAngleFrame &) override { called = UART_SERVO_GET_ANGLE; }
        void visit(const bolt::EncoderGetValuesFrame &) override { called = ENCODER_GET_VALUES; }
        void visit(const bolt::ImuGetValuesFrame &) override { called = IMU_GET_VALUES; }
        void visit(const bolt::PidMotorSetRpmFrame &) override { called = PID_MOTOR_SET_RPM; }
        void visit(const bolt::PidMotorStopFrame &) override { called = PID_MOTOR_STOP; }
        void visit(const bolt::PidSetGainsFrame &) override { called = PID_SET_GAINS; }
    };
}

// --- PingFrame ---

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

// --- MotorSpeedFrame ---

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
    EXPECT_EQ(frame.motor, 0);
    EXPECT_EQ(frame.pulse, 0);
}

// --- MotorStopFrame ---

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

TEST(MotorStopFrameTest, DefaultFieldValues)
{
    bolt::MotorStopFrame frame;
    EXPECT_EQ(frame.brake, 0);
}

// --- PwmServoFrame ---

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
    EXPECT_EQ(frame.servo, 0);
    EXPECT_EQ(frame.angle, 0);
}

// --- UartServoFrame ---

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
    EXPECT_EQ(frame.servo, 0);
    EXPECT_EQ(frame.pulse, 0);
    EXPECT_EQ(frame.time, 0);
}

// --- UartServoGetAngleFrame ---

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

TEST(UartServoGetAngleFrameTest, DefaultFieldValues)
{
    bolt::UartServoGetAngleFrame frame;
    EXPECT_EQ(frame.servo, 0);
}

// --- EncoderGetValuesFrame ---

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

// --- ImuGetValuesFrame ---

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

// --- PidMotorSetRpmFrame ---

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
    EXPECT_EQ(frame.motor, 0);
    EXPECT_FLOAT_EQ(frame.rpm, 0.0f);
}

// --- PidMotorStopFrame ---

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
    EXPECT_EQ(frame.motor, 0);
    EXPECT_EQ(frame.brake, 0);
}

// --- PidSetGainsFrame ---

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
    EXPECT_EQ(frame.motor, 0);
    EXPECT_FLOAT_EQ(frame.kp, 0.0f);
    EXPECT_FLOAT_EQ(frame.ki, 0.0f);
    EXPECT_FLOAT_EQ(frame.kd, 0.0f);
}
