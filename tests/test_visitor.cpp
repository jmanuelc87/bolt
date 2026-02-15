// test_visitor.cpp
//
// Unit tests for bolt::AppVisitor (Bolt/Inc/bolt/visitor.hpp).
//
// Strategy
// --------
// visitor.hpp includes peripherals.hpp which normally drags in the entire HAL
// controller chain.  The stub at tests/stubs/peripherals.hpp (found first in
// the CMake include path) replaces it with lightweight mock controller classes
// that record every call without touching hardware.
//
// Queue output is captured through the modified cmsis_os2.h stub which stores
// the raw bytes written to osMessageQueuePut into g_lastQueueMessage.  Tests
// decode the captured Message to verify framing, type byte, and payload bytes.
//
// Global FreeRTOS queue handles are defined here as non-null sentinels so that
// the null-pointer guard in osMessageQueuePut is not hit.
//
// Include note
// ------------
// visitor.hpp is included AFTER the required FreeRTOS stub headers so that the
// preprocessor sees vTaskDelay / pdMS_TO_TICKS before the header is parsed.

#include <gtest/gtest.h>
#include <cstring>
#include <cstdint>

// Pull in FreeRTOS stubs (vTaskDelay, pdMS_TO_TICKS) before visitor.hpp
#include "cmsis_os.h"

// visitor.hpp includes frames.hpp, utils.h, and peripherals.hpp.
// The stub peripherals.hpp (tests/stubs/peripherals.hpp) shadows the real one.
#include "visitor.hpp"

// ---------------------------------------------------------------------------
// Required global definitions
// ---------------------------------------------------------------------------

// queues.hpp declares these extern; we must define them in exactly one TU.
osMessageQueueId_t processQueue = reinterpret_cast<osMessageQueueId_t>(1);
osMessageQueueId_t queryQueue   = reinterpret_cast<osMessageQueueId_t>(2);

// utils.h declares this extern "C"; define it here.
extern "C" osThreadId_t ledTaskHandle = nullptr;

// stm32f1xx_hal.h declares these extern; define them so the linker is happy.
TIM_TypeDef gStubTIM1 = {};
TIM_TypeDef gStubTIM8 = {};

// main.h declares these extern.
GPIO_TypeDef gStubGPIOB = {};
GPIO_TypeDef gStubGPIOC = {};
GPIO_TypeDef gStubGPIOD = {};

// spi.h declares this extern.
SPI_HandleTypeDef hspi2 = {};

// ---------------------------------------------------------------------------
// Helper: overlay the captured raw bytes onto the Message struct.
//
// Message layout (from queues.hpp):
//   uint16_t size        — number of bytes stored in data[]
//   uint8_t  data[38]    — serialised frame bytes
//
// Frame layout (from definitions.hpp / utils.h):
//   data[0]          = SOF  (0xAA)
//   data[1]          = TYPE
//   data[2]          = LEN
//   data[3..3+LEN-1] = PAYLOAD
//   data[3+LEN]      = CRC high
//   data[4+LEN]      = CRC low
//   data[5+LEN]      = EOF_ (0x55)
// ---------------------------------------------------------------------------
struct CapturedFrame
{
    uint16_t msg_size;   // from Message.size
    uint8_t  sof;
    uint8_t  type;
    uint8_t  len;
    uint8_t  payload[32];
    uint8_t  crc_hi;
    uint8_t  crc_lo;
    uint8_t  eof;
};

static CapturedFrame decodeCapture()
{
    CapturedFrame cf{};
    const uint8_t *raw = g_lastQueueMessage.raw;

    // raw[0..1] = Message.size (little-endian uint16_t)
    cf.msg_size = static_cast<uint16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));

    // raw[2..] = Message.data[]
    const uint8_t *frame = raw + 2;
    cf.sof    = frame[0];
    cf.type   = frame[1];
    cf.len    = frame[2];

    uint8_t pay_len = cf.len;
    if (pay_len > 32) pay_len = 32;
    memcpy(cf.payload, frame + 3, pay_len);

    cf.crc_hi = frame[3 + cf.len];
    cf.crc_lo = frame[4 + cf.len];
    cf.eof    = frame[5 + cf.len];

    return cf;
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class AppVisitorTest : public ::testing::Test
{
protected:
    // One fresh mock instance per controller type per test.
    bolt::controller::MotorController     mockMotor;
    bolt::controller::PWMServoController  mockPwmServo;
    bolt::controller::UartServoController mockUartServo;
    bolt::controller::EncoderController   mockEncoder;
    bolt::controller::ICM20948Controller  mockImu;
    bolt::controller::PIDMotorController  mockPid[4];

    bolt::AppVisitor visitor;

    void SetUp() override
    {
        // Wire global pointers to local mocks.
        gMotorController  = &mockMotor;
        gPwmServo         = &mockPwmServo;
        gUartServo        = &mockUartServo;
        gEncoderController = &mockEncoder;
        gImuController    = &mockImu;
        for (int i = 0; i < 4; ++i)
            gPidMotorController[i] = &mockPid[i];

        // Clear capture buffer before each test.
        g_lastQueueMessage = {};
    }

    void TearDown() override
    {
        // Null out globals so dangling pointers never escape fixture scope.
        gMotorController   = nullptr;
        gPwmServo          = nullptr;
        gUartServo         = nullptr;
        gEncoderController = nullptr;
        gImuController     = nullptr;
        for (int i = 0; i < 4; ++i)
            gPidMotorController[i] = nullptr;
    }
};

// ===========================================================================
// PingFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitPingFrame_SendsPongResponseWithOkPayload)
{
    bolt::PingFrame frame;

    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured) << "Expected a message to be queued";
    CapturedFrame cf = decodeCapture();

    EXPECT_EQ(cf.sof,  SOF)   << "Frame must start with SOF 0xAA";
    EXPECT_EQ(cf.type, PONG)  << "Response type must be PONG (0x01)";
    EXPECT_EQ(cf.eof,  EOF_)  << "Frame must end with EOF 0x55";

    // Payload must be the three bytes of "OK!" (no null terminator in frame)
    ASSERT_EQ(cf.len, 3u);
    EXPECT_EQ(cf.payload[0], 'O');
    EXPECT_EQ(cf.payload[1], 'K');
    EXPECT_EQ(cf.payload[2], '!');
}

TEST_F(AppVisitorTest, VisitPingFrame_DoesNotTouchMotorController)
{
    bolt::PingFrame frame;
    visitor.visit(frame);
    EXPECT_EQ(mockMotor.call_count_setSpeed, 0);
    EXPECT_EQ(mockMotor.call_count_stop,     0);
}

// ===========================================================================
// MotorSpeedFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_CallsSetSpeedWithCorrectArgs)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 2;
    frame.pulse = 1500;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_setSpeed, 1);
    EXPECT_EQ(mockMotor.last_setSpeed_motor, 2);
    EXPECT_EQ(mockMotor.last_setSpeed_pulse, 1500);
}

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_DoesNotCallStop)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 1;
    frame.pulse = 500;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_stop, 0);
}

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_NegativePulseForwarded)
{
    // Negative pulses represent reverse direction; visitor must not clamp them.
    bolt::MotorSpeedFrame frame;
    frame.motor = 3;
    frame.pulse = static_cast<int16_t>(-800);

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.last_setSpeed_pulse, static_cast<int16_t>(-800));
}

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_DoesNotQueueResponse)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 1;
    frame.pulse = 0;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_MotorId4Forwarded)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 4;
    frame.pulse = 2000;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.last_setSpeed_motor, 4u);
    EXPECT_EQ(mockMotor.last_setSpeed_pulse, 2000);
}

// ===========================================================================
// MotorStopFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitMotorStopFrame_CallsStopWithCorrectArgs)
{
    bolt::MotorStopFrame frame;
    frame.motor = 1;
    frame.brake = 1;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_stop,   1);
    EXPECT_EQ(mockMotor.last_stop_motor,   1u);
    EXPECT_EQ(mockMotor.last_stop_brake,   1u);
}

TEST_F(AppVisitorTest, VisitMotorStopFrame_DoesNotCallSetSpeed)
{
    bolt::MotorStopFrame frame;
    frame.motor = 2;
    frame.brake = 0;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_setSpeed, 0);
}

TEST_F(AppVisitorTest, VisitMotorStopFrame_BrakeZeroForwarded)
{
    bolt::MotorStopFrame frame;
    frame.motor = 3;
    frame.brake = 0;

    visitor.visit(frame);

    EXPECT_EQ(mockMotor.last_stop_brake, 0u);
}

TEST_F(AppVisitorTest, VisitMotorStopFrame_DoesNotQueueResponse)
{
    bolt::MotorStopFrame frame;
    frame.motor = 1;
    frame.brake = 1;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// PwmServoFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitPwmServoFrame_CallsSetAngleWithCorrectArgs)
{
    bolt::PwmServoFrame frame;
    frame.servo = 2;
    frame.angle = 90;

    visitor.visit(frame);

    EXPECT_EQ(mockPwmServo.call_count_setAngle, 1);
    EXPECT_EQ(mockPwmServo.last_servo,          2u);
    EXPECT_EQ(mockPwmServo.last_angle,          90u);
}

TEST_F(AppVisitorTest, VisitPwmServoFrame_AngleZeroForwarded)
{
    bolt::PwmServoFrame frame;
    frame.servo = 0;
    frame.angle = 0;

    visitor.visit(frame);

    EXPECT_EQ(mockPwmServo.last_angle, 0u);
}

TEST_F(AppVisitorTest, VisitPwmServoFrame_AngleMaxByteForwarded)
{
    bolt::PwmServoFrame frame;
    frame.servo = 3;
    frame.angle = 255;

    visitor.visit(frame);

    EXPECT_EQ(mockPwmServo.last_angle, 255u);
}

TEST_F(AppVisitorTest, VisitPwmServoFrame_DoesNotQueueResponse)
{
    bolt::PwmServoFrame frame;
    frame.servo = 1;
    frame.angle = 45;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// UartServoFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitUartServoFrame_CallsSetControlWithCorrectArgs)
{
    bolt::UartServoFrame frame;
    frame.servo = 3;
    frame.pulse = 2000;
    frame.time  = 500;

    visitor.visit(frame);

    EXPECT_EQ(mockUartServo.call_count_setControl,    1);
    EXPECT_EQ(mockUartServo.last_setControl_servo,    3u);
    EXPECT_EQ(mockUartServo.last_setControl_pulse, 2000u);
    EXPECT_EQ(mockUartServo.last_setControl_time,   500u);
}

TEST_F(AppVisitorTest, VisitUartServoFrame_MinPulseForwarded)
{
    bolt::UartServoFrame frame;
    frame.servo = 1;
    frame.pulse = 96;
    frame.time  = 0;

    visitor.visit(frame);

    EXPECT_EQ(mockUartServo.last_setControl_pulse, 96u);
}

TEST_F(AppVisitorTest, VisitUartServoFrame_MaxPulseForwarded)
{
    bolt::UartServoFrame frame;
    frame.servo = 6;
    frame.pulse = 4000;
    frame.time  = 1000;

    visitor.visit(frame);

    EXPECT_EQ(mockUartServo.last_setControl_pulse, 4000u);
}

TEST_F(AppVisitorTest, VisitUartServoFrame_DoesNotQueueResponse)
{
    bolt::UartServoFrame frame;
    frame.servo = 2;
    frame.pulse = 500;
    frame.time  = 300;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// UartServoGetAngleFrame — servo ready on first poll
// ===========================================================================

TEST_F(AppVisitorTest, VisitUartServoGetAngle_CallsSetControlAngleWithCorrectServoId)
{
    mockUartServo.ready_return = true;
    mockUartServo.angle_return = 2100;

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 4;

    visitor.visit(frame);

    EXPECT_EQ(mockUartServo.call_count_setControlAngle,    1);
    EXPECT_EQ(mockUartServo.last_setControlAngle_servo,    4u);
}

TEST_F(AppVisitorTest, VisitUartServoGetAngle_WhenReady_SendsComputedAngleResponse)
{
    // NOTE: The production computation uses integer division:
    //   int pwm = getAngle();  (uint16_t promoted to int)
    //   float percent = (pwm - 100) / 3800;  <-- integer division!
    //   float angle = 180 * percent;
    //
    // For pwm=2100: (2100-100)/3800 = 2000/3800 = 0 (int div), angle = 0.0f
    // For pwm=3900: (3900-100)/3800 = 3800/3800 = 1 (int div), angle = 180.0f
    //
    // We test what the code actually does, not what it may intend to do.
    mockUartServo.ready_return = true;
    mockUartServo.angle_return = 3900;  // (3900-100)/3800 = 1, angle = 180.0f

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 1;

    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured) << "Expected ANGLE response in queue";
    CapturedFrame cf = decodeCapture();

    EXPECT_EQ(cf.sof,  SOF)   << "Frame SOF must be 0xAA";
    EXPECT_EQ(cf.type, ANGLE) << "Response type must be ANGLE (0x03)";
    EXPECT_EQ(cf.eof,  EOF_)  << "Frame EOF must be 0x55";

    // Payload is a single float (4 bytes).
    EXPECT_EQ(cf.len, 4u) << "Payload length must be sizeof(float)=4";
    float angle_received;
    memcpy(&angle_received, cf.payload, sizeof(float));
    // (3900-100)/3800 = 1 (integer division), 180*1 = 180.0f
    EXPECT_FLOAT_EQ(angle_received, 180.0f);
}

TEST_F(AppVisitorTest, VisitUartServoGetAngle_WhenReadyImmediately_DoesNotPollMoreThanNeeded)
{
    // isReady() returns true on the very first call.
    mockUartServo.ready_return = true;
    mockUartServo.angle_return = 2000;

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 2;

    visitor.visit(frame);

    // The loop body executes: --i (=9), then checks !isReady() → false (ready).
    // So exactly one call to isReady().
    EXPECT_EQ(mockUartServo.call_count_isReady, 1);
}

TEST_F(AppVisitorTest, VisitUartServoGetAngle_WhenNeverReady_SendsNegativeOneAngle)
{
    // isReady always returns false — simulates timeout.
    mockUartServo.ready_return = false;
    mockUartServo.angle_return = 0;

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 5;

    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured) << "Expected ANGLE response even on timeout";
    CapturedFrame cf = decodeCapture();

    EXPECT_EQ(cf.type, ANGLE);
    ASSERT_EQ(cf.len, 4u);
    float angle_received;
    memcpy(&angle_received, cf.payload, sizeof(float));
    EXPECT_FLOAT_EQ(angle_received, -1.0f) << "Timeout must send -1.0 as angle";
}

TEST_F(AppVisitorTest, VisitUartServoGetAngle_WhenNeverReady_PollsExactlyNineTimes)
{
    // The visitor loop: i=10, --i makes i=9,8,...,1; exits when i==0.
    // While loop condition: --i > 0 && !isReady()
    // i starts at 10; after decrement on first iteration: i=9, check !ready (true) → body
    // continues until i=1, --i=0, loop exits.
    // isReady is called 9 times.
    mockUartServo.ready_return = false;

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 1;

    visitor.visit(frame);

    EXPECT_EQ(mockUartServo.call_count_isReady, 9)
        << "Loop polls isReady exactly 9 times before timeout (i counts 9..1)";
}

TEST_F(AppVisitorTest, VisitUartServoGetAngle_WhenReadyOnFirstPoll_SendsComputedNonNegativeAngle)
{
    // When isReady() returns true, the visitor takes the if(i>0) branch and
    // sends a computed angle, never -1.0.
    // angle_return=100 → (100-100)/3800=0 (int div) → angle=0.0f
    mockUartServo.ready_return = true;
    mockUartServo.angle_return = 100;

    bolt::UartServoGetAngleFrame frame;
    frame.servo = 3;

    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured);
    CapturedFrame cf = decodeCapture();
    float angle_received;
    memcpy(&angle_received, cf.payload, sizeof(float));
    // Computed angle is 0.0f, NOT -1.0f.
    EXPECT_FLOAT_EQ(angle_received, 0.0f);
    EXPECT_NE(angle_received, -1.0f) << "Ready path must not send -1.0 (the timeout sentinel)";
}

// ===========================================================================
// EncoderGetValuesFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitEncoderGetValues_CallsGetRPMForAllFourEncoders)
{
    bolt::EncoderGetValuesFrame frame;

    visitor.visit(frame);

    // getRPM must be called exactly four times (for ids 1..4).
    EXPECT_EQ(mockEncoder.call_count_getRPM, 4);
}

TEST_F(AppVisitorTest, VisitEncoderGetValues_SendsRpmsResponseWithFourFloats)
{
    mockEncoder.rpm_values[0] = 10.0f;
    mockEncoder.rpm_values[1] = 20.0f;
    mockEncoder.rpm_values[2] = 30.0f;
    mockEncoder.rpm_values[3] = 40.0f;

    bolt::EncoderGetValuesFrame frame;
    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured);
    CapturedFrame cf = decodeCapture();

    EXPECT_EQ(cf.sof,  SOF);
    EXPECT_EQ(cf.type, RPMS) << "Response type must be RPMS (0x02)";
    EXPECT_EQ(cf.eof,  EOF_);

    // 4 floats × 4 bytes = 16 bytes payload
    ASSERT_EQ(cf.len, 16u) << "RPMS payload must be 4 × sizeof(float)";

    float rpm[4];
    memcpy(rpm, cf.payload, sizeof(rpm));
    EXPECT_FLOAT_EQ(rpm[0], 10.0f);
    EXPECT_FLOAT_EQ(rpm[1], 20.0f);
    EXPECT_FLOAT_EQ(rpm[2], 30.0f);
    EXPECT_FLOAT_EQ(rpm[3], 40.0f);
}

TEST_F(AppVisitorTest, VisitEncoderGetValues_AllZeroRpmsEncodedCorrectly)
{
    // Default mock returns 0.0f for all encoders.
    bolt::EncoderGetValuesFrame frame;
    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured);
    CapturedFrame cf = decodeCapture();

    ASSERT_EQ(cf.len, 16u);
    float rpm[4];
    memcpy(rpm, cf.payload, sizeof(rpm));
    for (int i = 0; i < 4; ++i)
        EXPECT_FLOAT_EQ(rpm[i], 0.0f) << "Encoder " << (i + 1) << " RPM should be 0";
}

// ===========================================================================
// ImuGetValuesFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitImuGetValues_CallsReadAllOnce)
{
    bolt::ImuGetValuesFrame frame;

    visitor.visit(frame);

    EXPECT_EQ(mockImu.call_count_readAll, 1);
}

TEST_F(AppVisitorTest, VisitImuGetValues_SendsImuResponseWith20BytePayload)
{
    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    ASSERT_TRUE(g_lastQueueMessage.captured);
    CapturedFrame cf = decodeCapture();

    EXPECT_EQ(cf.sof,  SOF);
    EXPECT_EQ(cf.type, IMU) << "Response type must be IMU (0x04)";
    EXPECT_EQ(cf.eof,  EOF_);
    EXPECT_EQ(cf.len, 20u)  << "IMU payload is 10 × int16 = 20 bytes";
}

TEST_F(AppVisitorTest, VisitImuGetValues_AccelXEncodedAsBigEndian)
{
    // accel_x = 0x1234 → payload[0]=0x12, payload[1]=0x34
    mockImu.mock_data.accel_x = static_cast<int16_t>(0x1234);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[0], 0x12u);
    EXPECT_EQ(cf.payload[1], 0x34u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_AccelYEncodedAsBigEndian)
{
    mockImu.mock_data.accel_y = static_cast<int16_t>(0x5678);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[2], 0x56u);
    EXPECT_EQ(cf.payload[3], 0x78u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_AccelZEncodedAsBigEndian)
{
    mockImu.mock_data.accel_z = static_cast<int16_t>(0x0102);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[4], 0x01u);
    EXPECT_EQ(cf.payload[5], 0x02u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_GyroXEncodedAsBigEndian)
{
    mockImu.mock_data.gyro_x = static_cast<int16_t>(0xABCD);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[6], 0xABu);
    EXPECT_EQ(cf.payload[7], 0xCDu);
}

TEST_F(AppVisitorTest, VisitImuGetValues_GyroYEncodedAsBigEndian)
{
    mockImu.mock_data.gyro_y = static_cast<int16_t>(0xEF01);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[8],  0xEFu);
    EXPECT_EQ(cf.payload[9],  0x01u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_GyroZEncodedAsBigEndian)
{
    mockImu.mock_data.gyro_z = static_cast<int16_t>(0x0A0B);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[10], 0x0Au);
    EXPECT_EQ(cf.payload[11], 0x0Bu);
}

TEST_F(AppVisitorTest, VisitImuGetValues_MagXEncodedAsBigEndian)
{
    mockImu.mock_data.mag_x = static_cast<int16_t>(0x1122);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[12], 0x11u);
    EXPECT_EQ(cf.payload[13], 0x22u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_MagYEncodedAsBigEndian)
{
    mockImu.mock_data.mag_y = static_cast<int16_t>(0x3344);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[14], 0x33u);
    EXPECT_EQ(cf.payload[15], 0x44u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_MagZEncodedAsBigEndian)
{
    mockImu.mock_data.mag_z = static_cast<int16_t>(0x5566);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[16], 0x55u);
    EXPECT_EQ(cf.payload[17], 0x66u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_TemperatureEncodedAsBigEndian)
{
    mockImu.mock_data.temperature = static_cast<int16_t>(0x7788);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[18], 0x77u);
    EXPECT_EQ(cf.payload[19], 0x88u);
}

TEST_F(AppVisitorTest, VisitImuGetValues_NegativeTemperatureTwosComplement)
{
    // -1 = 0xFFFF → big-endian: 0xFF, 0xFF
    mockImu.mock_data.temperature = static_cast<int16_t>(-1);

    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.payload[18], 0xFFu);
    EXPECT_EQ(cf.payload[19], 0xFFu);
}

TEST_F(AppVisitorTest, VisitImuGetValues_AllFieldsZero_PayloadAllZeroes)
{
    // Default mock_data is zero-initialised.
    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    CapturedFrame cf = decodeCapture();
    for (int i = 0; i < 20; ++i)
        EXPECT_EQ(cf.payload[i], 0u) << "payload byte " << i << " should be 0";
}

// ===========================================================================
// PidMotorSetRpmFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_CallsSetTargetRPMOnCorrectController)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 2;     // visitor computes idx = motor - 1 = 1
    frame.rpm   = 150.0f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[1].call_count_setTargetRPM, 1);
    EXPECT_FLOAT_EQ(mockPid[1].last_setTargetRPM_rpm, 150.0f);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_Motor1MapsToIndex0)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 1;
    frame.rpm   = 200.0f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[0].call_count_setTargetRPM, 1);
    EXPECT_FLOAT_EQ(mockPid[0].last_setTargetRPM_rpm, 200.0f);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_Motor4MapsToIndex3)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 4;
    frame.rpm   = 300.0f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[3].call_count_setTargetRPM, 1);
    EXPECT_FLOAT_EQ(mockPid[3].last_setTargetRPM_rpm, 300.0f);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_NegativeRPMForwarded)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 1;
    frame.rpm   = -100.0f;

    visitor.visit(frame);

    EXPECT_FLOAT_EQ(mockPid[0].last_setTargetRPM_rpm, -100.0f);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_ZeroRPMForwarded)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 3;
    frame.rpm   = 0.0f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[2].call_count_setTargetRPM, 1);
    EXPECT_FLOAT_EQ(mockPid[2].last_setTargetRPM_rpm, 0.0f);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_OutOfBoundsMotorId0_NoControllerCalled)
{
    // motor=0 → idx=255 (uint8_t underflow), fails bounds check idx < 4
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 0;
    frame.rpm   = 100.0f;

    visitor.visit(frame); // must not crash or call any controller

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_setTargetRPM, 0) << "No PID should be called for motor=0";
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_OutOfBoundsMotorId5_NoControllerCalled)
{
    // motor=5 → idx=4, fails idx < 4
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 5;
    frame.rpm   = 100.0f;

    visitor.visit(frame);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_setTargetRPM, 0);
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_NullControllerSkippedGracefully)
{
    // Remove controller at index 1; visitor must skip it without crashing.
    gPidMotorController[1] = nullptr;

    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 2;   // idx = 1
    frame.rpm   = 50.0f;

    ASSERT_NO_FATAL_FAILURE(visitor.visit(frame));
}

TEST_F(AppVisitorTest, VisitPidMotorSetRpm_DoesNotQueueResponse)
{
    bolt::PidMotorSetRpmFrame frame;
    frame.motor = 1;
    frame.rpm   = 100.0f;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// PidMotorStopFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitPidMotorStop_CallsStopOnCorrectController)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 3;    // idx = 2
    frame.brake = 1;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[2].call_count_stop, 1);
    EXPECT_EQ(mockPid[2].last_stop_brake, 1u);
}

TEST_F(AppVisitorTest, VisitPidMotorStop_BrakeZeroForwarded)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 1;
    frame.brake = 0;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[0].last_stop_brake, 0u);
}

TEST_F(AppVisitorTest, VisitPidMotorStop_OutOfBoundsMotorId0_NoControllerCalled)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 0;   // idx=255, out of bounds
    frame.brake = 1;

    visitor.visit(frame);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_stop, 0);
}

TEST_F(AppVisitorTest, VisitPidMotorStop_OutOfBoundsMotorId5_NoControllerCalled)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 5;
    frame.brake = 0;

    visitor.visit(frame);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_stop, 0);
}

TEST_F(AppVisitorTest, VisitPidMotorStop_NullControllerSkippedGracefully)
{
    gPidMotorController[2] = nullptr;

    bolt::PidMotorStopFrame frame;
    frame.motor = 3;   // idx = 2
    frame.brake = 1;

    ASSERT_NO_FATAL_FAILURE(visitor.visit(frame));
}

TEST_F(AppVisitorTest, VisitPidMotorStop_DoesNotQueueResponse)
{
    bolt::PidMotorStopFrame frame;
    frame.motor = 1;
    frame.brake = 0;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// PidSetGainsFrame
// ===========================================================================

TEST_F(AppVisitorTest, VisitPidSetGains_CallsSetGainsOnCorrectController)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 1;
    frame.kp    = 1.5f;
    frame.ki    = 0.2f;
    frame.kd    = 0.05f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[0].call_count_setGains, 1);
    EXPECT_FLOAT_EQ(mockPid[0].last_setGains_kp, 1.5f);
    EXPECT_FLOAT_EQ(mockPid[0].last_setGains_ki, 0.2f);
    EXPECT_FLOAT_EQ(mockPid[0].last_setGains_kd, 0.05f);
}

TEST_F(AppVisitorTest, VisitPidSetGains_Motor4MapsToIndex3)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 4;
    frame.kp    = 2.0f;
    frame.ki    = 0.1f;
    frame.kd    = 0.01f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[3].call_count_setGains, 1);
    EXPECT_FLOAT_EQ(mockPid[3].last_setGains_kp, 2.0f);
}

TEST_F(AppVisitorTest, VisitPidSetGains_ZeroGainsForwarded)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 2;
    frame.kp    = 0.0f;
    frame.ki    = 0.0f;
    frame.kd    = 0.0f;

    visitor.visit(frame);

    EXPECT_EQ(mockPid[1].call_count_setGains, 1);
    EXPECT_FLOAT_EQ(mockPid[1].last_setGains_kp, 0.0f);
    EXPECT_FLOAT_EQ(mockPid[1].last_setGains_ki, 0.0f);
    EXPECT_FLOAT_EQ(mockPid[1].last_setGains_kd, 0.0f);
}

TEST_F(AppVisitorTest, VisitPidSetGains_OutOfBoundsMotorId0_NoControllerCalled)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 0;
    frame.kp    = 1.0f;
    frame.ki    = 1.0f;
    frame.kd    = 1.0f;

    visitor.visit(frame);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_setGains, 0);
}

TEST_F(AppVisitorTest, VisitPidSetGains_OutOfBoundsMotorId5_NoControllerCalled)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 5;
    frame.kp    = 1.0f;
    frame.ki    = 0.5f;
    frame.kd    = 0.1f;

    visitor.visit(frame);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_setGains, 0);
}

TEST_F(AppVisitorTest, VisitPidSetGains_NullControllerSkippedGracefully)
{
    gPidMotorController[0] = nullptr;

    bolt::PidSetGainsFrame frame;
    frame.motor = 1;   // idx = 0
    frame.kp    = 1.0f;
    frame.ki    = 0.0f;
    frame.kd    = 0.0f;

    ASSERT_NO_FATAL_FAILURE(visitor.visit(frame));
}

TEST_F(AppVisitorTest, VisitPidSetGains_DoesNotQueueResponse)
{
    bolt::PidSetGainsFrame frame;
    frame.motor = 1;
    frame.kp    = 1.0f;
    frame.ki    = 0.0f;
    frame.kd    = 0.0f;

    visitor.visit(frame);

    EXPECT_FALSE(g_lastQueueMessage.captured);
}

// ===========================================================================
// Visitor isolation: each visit only touches its own controller
// ===========================================================================

TEST_F(AppVisitorTest, VisitMotorSpeedFrame_DoesNotTouchOtherControllers)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 1;
    frame.pulse = 100;

    visitor.visit(frame);

    EXPECT_EQ(mockPwmServo.call_count_setAngle,     0);
    EXPECT_EQ(mockUartServo.call_count_setControl,  0);
    EXPECT_EQ(mockEncoder.call_count_getRPM,        0);
    EXPECT_EQ(mockImu.call_count_readAll,           0);
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(mockPid[i].call_count_setTargetRPM, 0);
}

TEST_F(AppVisitorTest, VisitEncoderGetValues_DoesNotTouchMotorOrPidControllers)
{
    bolt::EncoderGetValuesFrame frame;
    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_setSpeed, 0);
    EXPECT_EQ(mockMotor.call_count_stop,     0);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_EQ(mockPid[i].call_count_setTargetRPM, 0);
        EXPECT_EQ(mockPid[i].call_count_stop,         0);
    }
}

TEST_F(AppVisitorTest, VisitImuGetValues_DoesNotTouchMotorOrEncoderControllers)
{
    bolt::ImuGetValuesFrame frame;
    visitor.visit(frame);

    EXPECT_EQ(mockMotor.call_count_setSpeed,   0);
    EXPECT_EQ(mockEncoder.call_count_getRPM,   0);
}

// ===========================================================================
// Frame accept() dispatch integration: accept() calls the correct visit()
// ===========================================================================

TEST_F(AppVisitorTest, FrameAccept_MotorSpeedFrame_DispatchesToSetSpeed)
{
    bolt::MotorSpeedFrame frame;
    frame.motor = 2;
    frame.pulse = 500;

    frame.accept(visitor);

    EXPECT_EQ(mockMotor.call_count_setSpeed, 1);
    EXPECT_EQ(mockMotor.last_setSpeed_motor, 2u);
}

TEST_F(AppVisitorTest, FrameAccept_PingFrame_DispatchesToSendPong)
{
    bolt::PingFrame frame;
    frame.accept(visitor);

    ASSERT_TRUE(g_lastQueueMessage.captured);
    CapturedFrame cf = decodeCapture();
    EXPECT_EQ(cf.type, PONG);
}

TEST_F(AppVisitorTest, FrameAccept_EncoderFrame_DispatchesToGetRPM)
{
    bolt::EncoderGetValuesFrame frame;
    frame.accept(visitor);

    EXPECT_EQ(mockEncoder.call_count_getRPM, 4);
}