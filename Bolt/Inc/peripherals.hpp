#ifndef BOLT_PERIPHERALS_HPP
#define BOLT_PERIPHERALS_HPP

#include "controller/peripheral_controllers.hpp"
#include "interface/serial_interface.hpp"
#include "interface/pwm_interface.hpp"

using bolt::pwm::MotorInterface;
using bolt::serial::UartAsyncSerialPort;

UartAsyncSerialPort *gUart1 = nullptr;
MotorInterface *gMotors = nullptr;

extern "C" void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1);
    UartAsyncSerialPort::registry().insert({&huart1, &port});
    gUart1 = &port;

    static MotorInterface motors;
    gMotors = &motors;
}

#endif /* BOLT_PERIPHERALS_HPP */
