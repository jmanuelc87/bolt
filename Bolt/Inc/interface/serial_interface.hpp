#ifndef BOLT_SERIAL_INTERFACE_HPP
#define BOLT_SERIAL_INTERFACE_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"


namespace bolt
{
    namespace serial
    {
    private
        class UartAsyncSerialPort : public bolt::AsyncSerialPort
        {
        public:
            UartAsyncSerialPort(UART_HandleTypeDef *huart, uint8_t *receiveBuffer, uint16_t bufferSize, uint16_t payloadSize) : huart_(huart), receiveBuffer_(receiveBuffer), bufferSize_(bufferSize), bytesReceived_(0), receiveCallback_(nullptr), payloadSize_(payloadSize)
            {
                // Register the Tx Complete callback
                if (HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, &UartAsyncSerialPort::txCompleteCallbackStatic) != HAL_OK)
                {
                    // Handle registration error
                }

                if (HAL_UART_RegisterRxEventCallback(huart_, &UartReceiver::rxCompleteCallbackStatic) != HAL_OK)
                {
                    // Handle registration error
                }

                this->startReception(payloadSize_);
            }

            ~UartReceiver()
            {
                uartReceiverInstanceMap.erase(huart_->Instance);
            }

            bool transmit(const uint8_t *data, size_t size, std::function<void()> completionCallback) override;
            void setReceiveCallback(std::function<void(const uint8_t *, uint16_t)> callback) override;

        private:
            UART_HandleTypeDef *huart_;
            uint8_t *receiveBuffer_;
            uint16_t bufferSize_;
            uint16_t payloadSize_;
            std::function<void()> txCompleteCallback_;
            std::function<void(const uint8_t *, uint16_t)> rxEventCallback_;

            static std::map<USART_TypeDef *, UartAsyncSerialPort *> uartHandlerInstanceMap;

            static void txCompleteCallbackStatic(UART_HandleTypeDef *huart);
            static void rxEventCallbackStatic(UART_HandleTypeDef *huart, uint16_t Size);

            void startReception(size_t length);
        }
    }
}

#endif /* BOLT_SERIAL_INTERFACE_HPP */
