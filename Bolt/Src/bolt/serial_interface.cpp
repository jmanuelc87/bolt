#include "interface/serial_interface.hpp"

int bolt::serial::UartAsyncSerialPort::transmit(const uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(huart_, data, size);
    configASSERT(st == HAL_OK);

    return 0;
}

void bolt::serial::UartAsyncSerialPort::startReception()
{
    HAL_UARTEx_ReceiveToIdle_IT(huart_, receiveBuffer_, bufferSize_);
}

uint8_t *bolt::serial::UartAsyncSerialPort::getData()
{
    return this->receiveBuffer_;
}