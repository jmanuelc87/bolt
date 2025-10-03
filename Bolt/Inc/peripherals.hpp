#ifndef BOLT_PERIPHERALS_HPP
#define BOLT_PERIPHERALS_HPP

#include "interface/serial_interface.hpp"
#include "interface/timer_interface.hpp"

#include "controller/servo_controller.hpp"
#include "controller/motor_controller.hpp"
#include "controller/encoder_controller.hpp"

#include "definitions.hpp"

using bolt::controller::UartServoController;
using bolt::serial::UartAsyncSerialPort;

using bolt::controller::EncoderController;
using bolt::controller::MotorController;

using bolt::timer::CountAsyncTimerPort;
using bolt::timer::CountSyncTimerPort;
using bolt::timer::PWMSyncTimerPort;

UartAsyncSerialPort *gUart1 = nullptr;

UartServoController *gUartServo = nullptr;
MotorController *gMotorController = nullptr;
EncoderController *gEncoderController = nullptr;

extern "C" void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1, BUFF_SIZE);
    gUart1 = &port;

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

    static CountAsyncTimerPort asyncTimerPort1(&htim7);

    static CountSyncTimerPort syncTimerPort3(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort4(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort5(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    static CountSyncTimerPort syncTimerPort6(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    static EncoderController encoderController(&asyncTimerPort1, &syncTimerPort3, &syncTimerPort4, &syncTimerPort5, &syncTimerPort6);
    gEncoderController = &encoderController;
}

#endif /* BOLT_PERIPHERALS_HPP */
