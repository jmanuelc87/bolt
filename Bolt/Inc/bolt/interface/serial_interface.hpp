#ifndef BOLT_SERIAL_INTERFACE_HPP
#define BOLT_SERIAL_INTERFACE_HPP

#include <unordered_map>

#include "interface.hpp"
#include "registry/handle_registry.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "definitions.hpp"

using bolt::registry::HandleRegistry;

namespace bolt
{
    namespace serial
    {
        class UartAsyncSerialPort : public bolt::AsyncSerialPort
        {
        public:
            UartAsyncSerialPort(UART_HandleTypeDef *huart, uint8_t size) : huart_(huart), bufferSize_(size)
            {
                this->receiveBuffer_ = new uint8_t[this->bufferSize_];
                HandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>::registerCallbacks(huart_);
                HandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>::registry().insert({huart_, this});
            }

            ~UartAsyncSerialPort()
            {
                delete[] this->receiveBuffer_;
                HAL_UART_Abort_IT(huart_);
                HandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>::registry().erase(huart_);
            }

            int transmitAndForget(const uint8_t *data, uint16_t size);
            int transmit(const uint8_t *data, uint16_t size) override;
            void receiveToIdle();
            void receive(uint8_t size);
            uint8_t *getData();

            std::function<void()> txCompleteCallback;
            std::function<void(uint16_t)> rxEventCallback;
            std::function<void()> rxCompleteCallback;

            friend class HandleRegistry<UartAsyncSerialPort, UART_HandleTypeDef>;

        protected:
            UART_HandleTypeDef *huart_;
            uint8_t *receiveBuffer_;
            uint16_t bufferSize_;
        };
    }
}

#endif /* BOLT_SERIAL_INTERFACE_HPP */
