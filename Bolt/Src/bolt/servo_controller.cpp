#include "controller/servo_controller.hpp"

#include <cstdio>

#include "cstring"
#include "gpio.h"
#include "utils.h"

// bool bolt::controller::ServoController::setAngle(int16_t angle, uint8_t servo_id)
// {
//     float us = this->PwmServoAngleToUs(angle);

//     this->g_pwm_pulse[servo_id] = us;
//     return 1;
// }

// float bolt::controller::ServoController::PwmServoAngleToUs(uint8_t angle)
// {
//     static constexpr float MIN_US = 500.0f;
//     static constexpr float MAX_US = 2400.0f;
//     static constexpr float NEUTRAL_US = 1500.0f;

//     // Accept either -90..+90 (centered) or 0..180 (absolute)
//     // If given 0..180, convert to centered around 0 at 90 deg.
//     float deg = static_cast<float>(angle);
//     if (deg >= 0.0f && deg <= 180.0f)
//     {
//         deg -= 90.0f; // now -90..+90
//     }

//     const float half_span = (MAX_US - MIN_US) * 0.5f; // ≈ 950 us
//     float us = NEUTRAL_US + deg * (half_span / 90.0f);

//     // Hard clamp to the servo's safe electrical range
//     if (us < MIN_US)
//         us = MIN_US;
//     if (us > MAX_US)
//         us = MAX_US;

//     return us;
// }

// void bolt::controller::ServoController::zeroFrameTicks()
// {
//     GPIOC->BSRR = (GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS2 | GPIO_BSRR_BS3);

//     // Convert pulse width (us) to countdown ticks
//     for (int i = 0; i < 4; ++i)
//     {
//         uint32_t us = g_pwm_pulse[i];
//         uint32_t ticks = (us + (STEP_US / 2)) / STEP_US; // round
//         if (ticks >= FRAME_TICKS)
//             ticks = FRAME_TICKS - 1;
//         ch_ticks[i] = static_cast<uint16_t>(ticks);
//     }

//     frame_ticks = FRAME_TICKS;
// }

// void bolt::controller::ServoController::decrementPerChannel()
// {
//     // Decrement per-channel and drop pins low when they expire
//     for (int i = 0; i < 4; i++)
//     {
//         if (ch_ticks[i] > 0)
//         {
//             ch_ticks[i]--;
//             if (ch_ticks[i] == 0)
//             {
//                 switch (i)
//                 {
//                 case 0:
//                     GPIOC->BSRR = GPIO_BSRR_BR0;
//                     break;
//                 case 1:
//                     GPIOC->BSRR = GPIO_BSRR_BR1;
//                     break;
//                 case 2:
//                     GPIOC->BSRR = GPIO_BSRR_BR2;
//                     break;
//                 case 3:
//                     GPIOC->BSRR = GPIO_BSRR_BR3;
//                     break;
//                 }
//             }
//         }
//     }

//     if (frame_ticks > 0)
//         frame_ticks--;
// }

bolt::controller::UartServoController::UartServoController(UART_HandleTypeDef *huart, uint8_t size) : bolt::serial::UartAsyncSerialPort(huart, size)
{
    rxCompleteCallback = [this]()
    {
        uint8_t *data = this->getData();

        if (this->receiveData(data[0]))
        {
            ready_ = true;
        }
        this->receive(1);
    };
    this->receive(1);
}

bool bolt::controller::UartServoController::receiveData(uint8_t byte)
{
    switch (state_)
    {
    case S_WAIT_SOF:
        if (byte == 0xff)
        {
            buff_[0] = 0xff;
            state_ = S_TYPE;
        }
        else if (byte == 0xf5)
        {
            buff_[0] = 0xff;
            buff_[1] = 0xf5;
            state_ = S_PAYLOAD;
            idx_ = 2;
        }
        break;

    case S_TYPE:
        if (byte == 0xF5)
        {
            buff_[1] = 0xF5;
            idx_ = 2;
            state_ = S_PAYLOAD;
        }
        else
        {
            this->reset();
            state_ = S_TYPE;
        }
        break;

    case S_PAYLOAD:
        buff_[idx_] = byte;
        idx_++;
        if (idx_ >= 8)
        {
            state_ = S_WAIT_SOF;
            return true;
        }
        break;
    }
    return false;
}

void bolt::controller::UartServoController::setControl(uint8_t id, uint16_t pulse, uint16_t time)
{
    uint8_t s_id = id & 0xFF;
    uint8_t len = 0x07;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x2A;

    if (pulse >= MAX_PULSE)
        pulse = MEDIAN_VALUE;
    else if (pulse < MIN_PULSE)
        pulse = MEDIAN_VALUE;

    uint8_t pos_h = (pulse >> 8) & 0xFF;
    uint8_t pos_l = pulse & 0xFF;

    uint8_t time_h = (time >> 8) & 0xFF;
    uint8_t time_l = time & 0xFF;

    uint8_t checksum = (~(s_id + len + cmd + addr + pos_h + pos_l + time_h + time_l)) & 0xFF;
    uint8_t data[] = {0xFF, 0xFF, s_id, len, cmd, addr, pos_h, pos_l, time_h, time_l, checksum};

    this->transmitAndForget(data, sizeof(data));
}

void bolt::controller::UartServoController::setControlAngle(uint8_t id)
{
    uint8_t s_id = id & 0xFF;
    uint8_t len = 0x04;
    uint8_t cmd = 0x02;
    uint8_t param_h = 0x38;
    uint8_t param_l = 0x02;

    uint8_t checksum = (~(s_id + len + cmd + param_h + param_l)) & 0xFF;
    uint8_t data[] = {0xFF, 0xFF, s_id, len, cmd, param_h, param_l, checksum};

    this->transmitAndForget(data, sizeof(data));
}

void bolt::controller::UartServoController::reset()
{
    idx_ = 0;
    buff_[0] = 0x00;
    buff_[1] = 0x00;
    buff_[2] = 0x00;
    buff_[3] = 0x00;
    buff_[4] = 0x00;
    buff_[5] = 0x00;
}

uint16_t bolt::controller::UartServoController::getAngle()
{

    uint16_t value = cur_[3] << 8 | cur_[4];
    return value;
}

bool bolt::controller::UartServoController::isReady()
{
    return ready_;
}

void bolt::controller::UartServoController::setReady(bool st)
{
    ready_ = st;
}