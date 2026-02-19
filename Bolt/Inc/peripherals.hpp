#ifndef BOLT_PERIPHERALS_HPP
#define BOLT_PERIPHERALS_HPP

#include "interface/serial_interface.hpp"
#include "interface/timer_interface.hpp"
#include "interface/can_interface.hpp"

#include "controller/servo_controller.hpp"
#include "controller/motor_controller.hpp"
#include "controller/encoder_controller.hpp"
#include "controller/icm20948_controller.hpp"
#include "controller/pid_motor_controller.hpp"
#include "controller/flash_controller.hpp"
#include "interface/flash_interface.hpp"

#include "definitions.hpp"
#include "main.h"
#include "spi.h"

using bolt::controller::UartServoController;
using bolt::serial::UartAsyncSerialPort;

using bolt::controller::EncoderController;
using bolt::controller::ICM20948Controller;
using bolt::controller::MotorController;
using bolt::controller::PIDMotorController;
using bolt::controller::PWMServoController;

using bolt::timer::CountAsyncTimerPort;
using bolt::timer::CountSyncTimerPort;
using bolt::timer::PROC_HandleTypeDef;
using bolt::timer::PWMSyncTimerPort;

using bolt::can::CanBusAsyncPort;
using bolt::controller::FlashController;
using bolt::controller::FlashKey;
using bolt::flash::InternalFlash;
using bolt::pin::GpioOutputPin;
using bolt::spi::SpiSyncPort;

UartAsyncSerialPort *gUart1 = nullptr;

CanBusAsyncPort *gCanBus = nullptr;

UartServoController *gUartServo = nullptr;
PWMServoController *gPwmServo = nullptr;
MotorController *gMotorController = nullptr;
EncoderController *gEncoderController = nullptr;
ICM20948Controller *gImuController = nullptr;
PIDMotorController *gPidMotorController[4] = {nullptr, nullptr, nullptr, nullptr};
FlashController *gFlashController = nullptr;
bolt::BatteryMonitor *gBatteryMonitor = nullptr;

extern "C" void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1, BUFF_SIZE);
    gUart1 = &port;

    static CanBusAsyncPort canPort(&hcan);
    gCanBus = &canPort;

    static UartServoController servoPort(&huart3, 10);
    gUartServo = &servoPort;

    int32_t ch_pwm1[4] = {TIM_CHANNEL_1, TIM_CHANNEL_4, -1, -1};
    int32_t ch_pwmn1[4] = {TIM_CHANNEL_2, TIM_CHANNEL_3, -1, -1};

    static PWMSyncTimerPort syncTimerPort1(&htim1, ch_pwm1, ch_pwmn1);

    int32_t ch_pwm2[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    int32_t ch_pwmn2[4] = {-1, -1, -1, -1};

    static PWMSyncTimerPort syncTimerPort2(&htim8, ch_pwm2, ch_pwmn2);

    static MotorController motorController(&syncTimerPort1, &syncTimerPort2);
    gMotorController = &motorController;

    int32_t ch_pwm_servo[4] = {-1, -1, -1, -1};
    int32_t ch_pwmn_servo[4] = {-1, -1, -1, -1};

    static PWMSyncTimerPort syncTimerPortServo(&htim7, ch_pwm_servo, ch_pwmn_servo);

    static GpioOutputPin servoPin0(S1_GPIO_Port, S1_Pin);
    static GpioOutputPin servoPin1(S2_GPIO_Port, S2_Pin);
    static GpioOutputPin servoPin2(S3_GPIO_Port, S3_Pin);
    static GpioOutputPin servoPin3(S4_GPIO_Port, S4_Pin);

    static PWMServoController pwmServoController(&syncTimerPortServo,
                                                 &servoPin0, &servoPin1, &servoPin2, &servoPin3);
    gPwmServo = &pwmServoController;

    HAL_TIM_Base_Start_IT(&htim7);

    static PROC_HandleTypeDef ptim1;
    ptim1.timer = 100;
    ptim1.counter = 100;

    static ProcessAsyncTimerPort procAsyncTimerPort(&ptim1);

    static CountSyncTimerPort syncTimerPort3(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort4(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort5(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort6(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    static EncoderController encoderController(&procAsyncTimerPort, &syncTimerPort3, &syncTimerPort4, &syncTimerPort5, &syncTimerPort6);
    gEncoderController = &encoderController;

    static GpioOutputPin imuCsPin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin);
    static SpiSyncPort spiPort(&hspi2, &imuCsPin);
    static ICM20948Controller imuController(&spiPort);
    gImuController = &imuController;

    static InternalFlash internalFlash;
    static FlashController flashController(&internalFlash, 0x0803FC00UL);
    gFlashController = &flashController;

    static PROC_HandleTypeDef pidTimHandles[4];
    static ProcessAsyncTimerPort *pidSamplers[4];
    static PIDMotorController *pidControllers[4];

    /* FlashKey layout: MOTOR1_KP=0, MOTOR1_KI=1, MOTOR1_KD=2, MOTOR2_KP=3, ... */
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t base = i * 3;
        float kp = flashController.load(static_cast<FlashKey>(base + 0));
        float ki = flashController.load(static_cast<FlashKey>(base + 1));
        float kd = flashController.load(static_cast<FlashKey>(base + 2));

        pidTimHandles[i].timer = 20;
        pidTimHandles[i].counter = 20;
        pidSamplers[i] = new ProcessAsyncTimerPort(&pidTimHandles[i]);
        pidControllers[i] = new PIDMotorController(
            pidSamplers[i],
            kp, ki, kd, 0.1f,
            -2000.0f, 2000.0f,
            &motorController, &encoderController,
            i + 1);
        gPidMotorController[i] = pidControllers[i];
    }
}

#endif /* BOLT_PERIPHERALS_HPP */
