#ifndef BOLT_SERIAL_PORT_HPP
#define BOLT_SERIAL_PORT_HPP

#include "main.h"

#include "interface/serial_interface.hpp"

using bolt::serial::UartAsyncSerialPort;

extern UartAsyncSerialPort *gUart1;

extern "C" void AppPeripheralsInit();

#endif /* BOLT_SERIAL_PORT_HPP */
