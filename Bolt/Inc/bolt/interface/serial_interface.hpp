#ifndef BOLT_SERIAL_INTERFACE_HPP
#define BOLT_SERIAL_INTERFACE_HPP

#include <unordered_map>

#include "interface.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

namespace bolt
{
    namespace serial
    {
        class UartAsyncSerialPort : public bolt::AsyncSerialPort
        {
        public:
            UartAsyncSerialPort(UART_HandleTypeDef *huart, uint16_t bufferSize, uint16_t payloadSize) : huart_(huart), bufferSize_(bufferSize), payloadSize_(payloadSize)
            {
                // Register the Tx Complete callback
                if (HAL_UART_RegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID, &C_TxCplt) != HAL_OK)
                {
                    // Handle registration error
                }

                if (HAL_UART_RegisterRxEventCallback(huart_, &C_RxCplt) != HAL_OK)
                {
                    // Handle registration error
                }

                this->receiveBuffer_ = new uint8_t[this->bufferSize_];
            }

            ~UartAsyncSerialPort()
            {
                delete[] this->receiveBuffer_;
                HAL_UART_Abort_IT(huart_);
                HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
                HAL_UART_UnRegisterRxEventCallback(huart_);
                UartAsyncSerialPort::registry().erase(huart_);
            }

            int transmit(const uint8_t *data, uint16_t size) override;
            void startReception(size_t length);

            std::function<void()> txCompleteCallback;
            std::function<void(uint16_t)> rxEventCallback;

            static std::unordered_map<UART_HandleTypeDef *, UartAsyncSerialPort *> &registry()
            {
                static std::unordered_map<UART_HandleTypeDef *, UartAsyncSerialPort *> r;
                return r;
            }

            static UartAsyncSerialPort *from(UART_HandleTypeDef *h)
            {
                auto it = registry().find(h);
                return (it == registry().end()) ? nullptr : it->second;
            }

        private:
            UART_HandleTypeDef *huart_;
            uint8_t *receiveBuffer_;
            uint16_t bufferSize_;
            uint16_t payloadSize_;

            static void C_TxCplt(UART_HandleTypeDef *h);
            static void C_RxCplt(UART_HandleTypeDef *h, uint16_t Size);
            static void C_Error(UART_HandleTypeDef *h);
        };
    }
}

#endif /* BOLT_SERIAL_INTERFACE_HPP */
