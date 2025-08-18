#include "serial_port.hpp"
#include "usart.h"

UartAsyncSerialPort *gUart1 = nullptr;

void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1, /*rxBuf*/ 100, /*txQ*/ 10);
    UartAsyncSerialPort::registry().insert({&huart1, &port});
    // If needed: port.beginRx(); port.registerCallbacks(); etc.
    gUart1 = &port;
}