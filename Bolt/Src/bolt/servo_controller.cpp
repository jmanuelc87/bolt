#include "controller/servo_controller.hpp"

#include <cstdio>

#include "cstring"
#include "gpio.h"
#include "utils.h"

bolt::controller::PWMServoController::PWMServoController(
    bolt::timer::PWMSyncTimerPort *port,
    GpioOutputPin *pin0, GpioOutputPin *pin1,
    GpioOutputPin *pin2, GpioOutputPin *pin3)
    : port_(port)
{
    pins_[0] = pin0;
    pins_[1] = pin1;
    pins_[2] = pin2;
    pins_[3] = pin3;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (pins_[i])
            pins_[i]->setLow();
    }

    port_->timElapsedCompleteCallback = [this]()
    {
        this->tick();
    };
}

bolt::controller::PWMServoController::~PWMServoController()
{
    port_->timElapsedCompleteCallback = nullptr;
}

void bolt::controller::PWMServoController::tick()
{
    if (frameCounter_ == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            if (pins_[i] && pulseTicks_[i] > 0)
            {
                pins_[i]->setHigh();
            }
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (pins_[i] && frameCounter_ == pulseTicks_[i])
        {
            pins_[i]->setLow();
        }
    }

    frameCounter_++;
    if (frameCounter_ >= FRAME_TICKS)
    {
        frameCounter_ = 0;
    }
}

void bolt::controller::PWMServoController::setAngle(uint8_t servo_id, uint8_t angle)
{
    if (servo_id >= 4)
        return;

    pulseTicks_[servo_id] = usToTicks(angleToUs(angle));
}

void bolt::controller::PWMServoController::setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4)
{
    pulseTicks_[0] = usToTicks(pulse1);
    pulseTicks_[1] = usToTicks(pulse2);
    pulseTicks_[2] = usToTicks(pulse3);
    pulseTicks_[3] = usToTicks(pulse4);
}

uint8_t bolt::controller::PWMServoController::usToTicks(int16_t us)
{
    if (us <= 0)
        return 0;

    uint8_t ticks = static_cast<uint8_t>((us + 50) / 100);

    if (ticks < 5)
        ticks = 5;
    if (ticks > 24)
        ticks = 24;

    return ticks;
}

int16_t bolt::controller::PWMServoController::angleToUs(uint8_t angle)
{
    static constexpr float MIN_US = 500.0f;
    static constexpr float MAX_US = 2400.0f;
    static constexpr float NEUTRAL_US = 1500.0f;

    float deg = static_cast<float>(angle);
    if (deg > 180.0f)
        deg = 180.0f;

    deg -= 90.0f;

    const float half_span = (MAX_US - MIN_US) * 0.5f;
    float us = NEUTRAL_US + deg * (half_span / 90.0f);

    if (us < MIN_US)
        us = MIN_US;
    if (us > MAX_US)
        us = MAX_US;

    return static_cast<int16_t>(us);
}

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
        if (byte == 0xFF)
        {
            buff_[0] = 0xFF;
            state_ = S_TYPE;
        }
        else if (byte == 0xF5)
        {
            buff_[0] = 0xFF;
            buff_[1] = 0xF5;
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
            state_ = S_WAIT_SOF;
        }
        break;

    case S_PAYLOAD:
        buff_[idx_] = byte;
        idx_++;
        if (idx_ >= 8)
        {
            memcpy(curr_, buff_, 8);
            this->reset();
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

    uint16_t value = curr_[5] << 8 | curr_[6];
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