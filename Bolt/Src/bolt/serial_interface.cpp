#include "interface/serial_interface.hpp"

int bolt::serial::UartAsyncSerialPort::transmit(const uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(this->huart_, data, size);
    configASSERT(st == HAL_OK);

    return 0;
}

void bolt::serial::UartAsyncSerialPort::startReception()
{
    HAL_UARTEx_ReceiveToIdle_IT(this->huart_, this->receiveBuffer_, this->bufferSize_);
}

uint8_t *bolt::serial::UartAsyncSerialPort::getData()
{
    return this->receiveBuffer_;
}