#ifndef BOLT_SERVO_CONTROLLER_HPP
#define BOLT_SERVO_CONTROLLER_HPP

#include <unordered_map>
#include <cstdint>

#include "interface/pin_interface.hpp"
#include "interface/serial_interface.hpp"
#include "interface/uart_handle_registry.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "definitions.hpp"

#define MEDIAN_VALUE 2000

#define MAX_PULSE 4000
#define MIN_PULSE 96

#define MAX_SERVO_NUM 6

using bolt::pin::GpioOutputPin;
using bolt::serial::UartHandleRegistry;

namespace bolt
{
    namespace controller
    {
        // how often TIM7 interrupt fires (us)
        static constexpr uint16_t STEP_US = 50;
        static constexpr uint16_t FRAME_US = 20000; // 20ms frame
        static constexpr uint16_t FRAME_TICKS = FRAME_US / STEP_US;

        // per-channel countdowns
        static volatile uint16_t ch_ticks[4];
        static volatile uint16_t frame_ticks = FRAME_TICKS;

        class ServoController : public bolt::PWMTimer
        {
        public:
            ServoController(TIM_HandleTypeDef *htim);
            ~ServoController() {}

            bool setPulse(int16_t pulse) override;
            bool setAngle(int16_t angle, uint8_t servo_id);
            void periodElapsed();
            float PwmServoAngleToUs(uint8_t angle);

            std::function<void()> timElapsedCompleteCallback;

            friend class UartHandleRegistry<ServoController, UART_HandleTypeDef>;

        private:
            TIM_HandleTypeDef *htim_;
            float g_pwm_pulse[4];

            void zeroFrameTicks();
            void decrementPerChannel();
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

            bool receiveData(uint8_t current);

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
        };
    }
}

#endif /* BOLT_SERVO_CONTROLLER_HPP */
