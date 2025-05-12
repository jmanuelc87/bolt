#include "interface/serial_interface.hpp"

bool bolt::serial::UartAsyncSerialPort::transmit(const uint8_t *data, size_t size, std::function<void()> completionCallback)
{
    this->txCompleteCallback_ = completionCallback;
    return HAL_UART_Transmit_IT(huart_, const_cast<uint8_t *>(data), size) == HAL_OK;
}

void bolt::serial::UartAsyncSerialPort::setReceiveCallback(std::function<void(const uint8_t *, uint16_t)> callback)
{
    this->rxCompleteCallback_ = callback;
}

void bolt::serial::UartAsyncSerialPort::startReception(uint16_t length)
{
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart_, this->receiveBuffer_, length);
}

static void bolt::serial::UartAsyncSerialPort::txCompleteCallbackStatic(UART_HandleTypeDef *huart)
{
    if (uartHandlerInstanceMap.count(huart->Instance))
    {
        uartHandlerInstanceMap[huart->Instance]->txCompleteCallback_();
    }
}

static void bolt::serial::UartAsyncSerialPort::rxEventCallbackStatic(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (uartHandlerInstanceMap.count(huart->Instance))
    {
        uartHandlerInstanceMap[huart->Instance]->rxEventCallback_(this->receiveBuffer_, Size);
        uartHandlerInstanceMap[huart->Instance]->startReception(this->payloadSize_);
    }
}