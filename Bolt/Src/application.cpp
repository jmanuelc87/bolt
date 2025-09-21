#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os2.h"

#include <cstdio>
#include <string.h>

#include "interface/pin_interface.hpp"
#include "controller/led_controller.hpp"
#include "interface/serial_interface.hpp"

#include "queues.hpp"
#include "frames.hpp"
#include "parser.hpp"
#include "visitor.hpp"

using bolt::AppVisitor;
using bolt::Frame;
using bolt::FrameDecoder;
using bolt::FrameParser;
using bolt::RawFrame;
using bolt::controller::LedController;
using bolt::pin::GpioOutputPin;
using bolt::serial::UartAsyncSerialPort;

extern "C" osThreadId_t ledTaskHandle;
extern "C" osThreadId_t commandTaskHandle;
extern "C" osThreadId_t processTaskHandle;

extern "C" void AppPeripheralsInit();

UartAsyncSerialPort *gUart1 = nullptr;

extern "C" void vLed_Task(void *argument)
{
    (void)argument;
    GpioOutputPin led(LED_GPIO_Port, LED_Pin);
    LedController led_controller(led);

    led.setLow();

    while (1)
    {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        led_controller.blink(2, 150);
    }
}

extern "C" void vProcess_Task(void *argument)
{
    (void)argument;

    FrameParser parser;
    FrameDecoder decoder;
    AppVisitor visitor;

    Message m;

    while (1)
    {
        if (osMessageQueueGet(processQueue, &m, NULL, osWaitForever) == osOK)
        {
            RawFrame rf;

            for (size_t i = 0; i < m.size; i++)
            {
                if (parser.push(m.data[i], rf))
                {
                    const Frame *f = decoder.decode(rf);
                    if (f)
                    {
                        f->accept(visitor);
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

extern "C" void vCommand_Task(void *argument)
{
    (void)argument;
    gUart1->rxEventCallback = [](uint16_t Size)
    {
        Message m;
        m.size = Size;
        memcpy(m.data, gUart1->getData(), Size);

        osStatus_t s = osMessageQueuePut(processQueue, &m, 0, 0);
        if (s == osOK)
            osThreadFlagsSet(ledTaskHandle, 0x01);

        gUart1->startReception();
    };

    gUart1->startReception();

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

    Message m;

    while (1)
    {
        if (osMessageQueueGet(queryQueue, &m, NULL, osWaitForever) == osOK)
        {
            gUart1->transmit(m.data, m.size);
        }
    }
}

extern "C" void AppPeripheralsInit()
{
    static UartAsyncSerialPort port(&huart1);
    UartAsyncSerialPort::registry().insert({&huart1, &port});
    gUart1 = &port;
}
