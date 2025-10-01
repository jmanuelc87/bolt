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

extern "C" void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1, BUFF_SIZE);
    gUart1 = &port;

    static UartServoController servoPort(&huart3, 10);
    gUartServo = &servoPort;

    PWMSyncTimerPort syncTimerPort1(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_4, TIM_CHANNEL_2 | TIM_CHANNEL_3);
    PWMSyncTimerPort syncTimerPort2(&htim8, TIM_CHANNEL_ALL, 0);

    static MotorController motorController(&syncTimerPort1, &syncTimerPort2);
    gMotorController = &motorController;
}

#endif /* BOLT_PERIPHERALS_HPP */
