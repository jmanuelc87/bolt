#ifndef BOLT_PERIPHERALS_HPP
#define BOLT_PERIPHERALS_HPP

#include "controller/peripheral_controllers.hpp"
#include "interface/serial_interface.hpp"
#include "controller/motor_controller.hpp"
#include "controller/servo_controller.hpp"

using bolt::controller::MOTOR_ID_M1;
using bolt::controller::MOTOR_ID_M2;
using bolt::controller::MOTOR_ID_M3;
using bolt::controller::MOTOR_ID_M4;
using bolt::controller::MotorController;
using bolt::controller::ServoController;
using bolt::controller::UartServoController;
using bolt::pin::GpioOutputPin;
using bolt::serial::UartAsyncSerialPort;

UartAsyncSerialPort *gUart1 = nullptr;

MotorController *gMotor1 = nullptr;
MotorController *gMotor2 = nullptr;
MotorController *gMotor3 = nullptr;
MotorController *gMotor4 = nullptr;

ServoController *gServo = nullptr;
UartServoController *gUartServo = nullptr;

extern "C" void AppPeripheralsInit()
{
    UartHandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>::registerCallbacks(&huart1);
    UartHandleRegistry<UartServoController, UART_HandleTypeDef>::registerCallbacks(&huart3);
    UartHandleRegistry<ServoController, TIM_HandleTypeDef>::registerCallbacks(&htim1);

    static UartAsyncSerialPort port(&huart1);
    UartHandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>::registry().insert({&huart1, &port});
    gUart1 = &port;

    static MotorController motor1(MOTOR_ID_M1);
    gMotor1 = &motor1;

    static MotorController motor2(MOTOR_ID_M2);
    gMotor2 = &motor2;

    static MotorController motor3(MOTOR_ID_M3);
    gMotor3 = &motor3;

    static MotorController motor4(MOTOR_ID_M4);
    gMotor4 = &motor4;

    static ServoController servo(&htim7);
    UartHandleRegistry<ServoController, TIM_HandleTypeDef>::registry().insert({&htim7, &servo});
    gServo = &servo;

    static UartServoController servoPort(&huart3);
    UartHandleRegistry<UartServoController, UART_HandleTypeDef>::registry().insert({&huart3, &servoPort});
    gUartServo = &servoPort;
}

#endif /* BOLT_PERIPHERALS_HPP */
