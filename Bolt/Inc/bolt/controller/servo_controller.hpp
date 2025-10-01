#ifndef BOLT_SERVO_CONTROLLER_HPP
#define BOLT_SERVO_CONTROLLER_HPP

#include <unordered_map>
#include <cstdint>

#include "interface/pin_interface.hpp"
#include "interface/serial_interface.hpp"
#include "registry/handle_registry.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "definitions.hpp"

#define MEDIAN_VALUE 2000

#define MAX_PULSE 4000
#define MIN_PULSE 96

#define MAX_SERVO_NUM 6

using bolt::pin::GpioOutputPin;
using bolt::registry::HandleRegistry;

namespace bolt
{
    namespace controller
    {
        class ServoController : public bolt::PWMTimer
        {
        };

        class UartServoController : public bolt::serial::UartAsyncSerialPort
        {
        public:
            UartServoController(UART_HandleTypeDef *huart, uint8_t size);

            void setControl(uint8_t id, uint16_t pulse, uint16_t time);
            void setControlAngle(uint8_t id);
            bool isReady();
            void setReady(bool st);
            uint16_t getAngle();

        private:
            enum State
            {
                S_WAIT_SOF,
                S_TYPE,
                S_PAYLOAD,
            } state_;

            uint8_t idx_;
            uint8_t cur_[8];
            uint8_t buff_[8];
            bool ready_ = false;

            void reset();
            bool receiveData(uint8_t current);
        };
    }
}

#endif /* BOLT_SERVO_CONTROLLER_HPP */
