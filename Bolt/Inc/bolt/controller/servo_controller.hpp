#ifndef BOLT_SERVO_CONTROLLER_HPP
#define BOLT_SERVO_CONTROLLER_HPP

#include <unordered_map>
#include <cstdint>

#include "interface/pin_interface.hpp"
#include "interface/serial_interface.hpp"
#include "interface/timer_interface.hpp"
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

        class PWMServoController : public ServoController
        {
        public:
            PWMServoController(bolt::timer::PWMSyncTimerPort *port,
                               GpioOutputPin *pin0, GpioOutputPin *pin1,
                               GpioOutputPin *pin2, GpioOutputPin *pin3);
            ~PWMServoController();

            void setAngle(uint8_t servo_id, uint8_t angle);

            void setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4) override;

        private:
            static constexpr uint16_t FRAME_TICKS = 200;

            bolt::timer::PWMSyncTimerPort *port_;
            GpioOutputPin *pins_[4];

            volatile uint8_t pulseTicks_[4] = {0, 0, 0, 0};
            volatile uint16_t frameCounter_ = 0;

            int16_t angleToUs(uint8_t angle);
            uint8_t usToTicks(int16_t us);
            void tick();
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
            uint8_t curr_[8];
            uint8_t buff_[8];
            bool ready_ = false;

            void reset();
            bool receiveData(uint8_t current);
        };
    }
}

#endif /* BOLT_SERVO_CONTROLLER_HPP */
