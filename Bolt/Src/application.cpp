#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os2.h"

#include "serial_port.hpp"
#include <cstdio>
#include <string.h>

#include "interface/pin_interface.hpp"
#include "controller/led_controller.hpp"

using bolt::controller::LedController;
using bolt::pin::GpioOutputPin;

extern UartAsyncSerialPort *gUart1;

extern "C" osThreadId_t ledTaskHandle;
extern "C" osThreadId_t commandTaskHandle;

typedef struct
{
    uint16_t size;
} UartMsg;

osMessageQueueId_t UartMsgQ;

extern "C" void AppQueuesInit(void)
{
    UartMsgQ = osMessageQueueNew(/*queue length*/ 8, /*item size*/ sizeof(UartMsg), NULL);
}

extern "C" void vLed_Task(void *argument)
{
    (void)argument;
    GpioOutputPin led(LED_GPIO_Port, LED_Pin);
    LedController led_controller(led);

    led.setLow();

    while (1)
    {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        led_controller.blink(2, 250);
    }
}

extern "C" void vCommand_Task(void *argument)
{
    (void)argument;
    gUart1->rxEventCallback = [](uint16_t Size)
    {
        gUart1->startReception(10);
        UartMsg m = {Size};
        osMessageQueuePut(UartMsgQ, &m, 0, 0);
        osThreadFlagsSet(ledTaskHandle, 0x01);
    };

    gUart1->startReception(10);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void vQuery_Task(void *argument)
{
    (void)argument;
    gUart1->txCompleteCallback = []
    {
        osThreadFlagsSet(ledTaskHandle, 0x01);
    };

    UartMsg m;
    char buff[100] = "";
    size_t len = 0;

    while (1)
    {
        if (osMessageQueueGet(UartMsgQ, &m, NULL, osWaitForever) == osOK)
        {
            sprintf(buff, "Received: %d\r\n", m.size);
            len = strlen(buff);

            gUart1->transmit(reinterpret_cast<const uint8_t *>(buff), len);
        }
    }
}
