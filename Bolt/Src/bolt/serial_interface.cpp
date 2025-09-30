#include "interface/serial_interface.hpp"

int bolt::serial::UartAsyncSerialPort::transmit(const uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(huart_, data, size);
    configASSERT(st == HAL_OK);

    return 0;
}

int bolt::serial::UartAsyncSerialPort::transmitAndForget(const uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit(huart_, data, size, 1000);
    configASSERT(st == HAL_OK);

    return 1;
}

void bolt::serial::UartAsyncSerialPort::receiveToIdle()
{
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_IT(huart_, receiveBuffer_, bufferSize_);
    configASSERT(st == HAL_OK);
}

void bolt::serial::UartAsyncSerialPort::receive(uint8_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Receive_IT(huart_, receiveBuffer_, size);
    configASSERT(st == HAL_OK);
}

uint8_t *bolt::serial::UartAsyncSerialPort::getData()
{
    return this->receiveBuffer_;
}
