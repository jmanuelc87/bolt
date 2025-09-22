#include "controller/servo_controller.hpp"

#include "gpio.h"

bool bolt::controller::ServoController::setPulse(int16_t pulse)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        this->g_pwm_pulse[i] = pulse;
    }

    return 1;
}

bool bolt::controller::ServoController::setAngle(int16_t angle, uint8_t servo_id)
{
    float us = this->PwmServo_Angle_To_Us(angle);

    this->g_pwm_pulse[servo_id] = us;
    return 1;
}

float bolt::controller::ServoController::PwmServo_Angle_To_Us(uint8_t angle)
{
    static constexpr float MIN_US = 500.0f;
    static constexpr float MAX_US = 2400.0f;
    static constexpr float NEUTRAL_US = 1500.0f;

    // Accept either -90..+90 (centered) or 0..180 (absolute)
    // If given 0..180, convert to centered around 0 at 90 deg.
    float deg = static_cast<float>(angle);
    if (deg >= 0.0f && deg <= 180.0f)
    {
        deg -= 90.0f; // now -90..+90
    }

    const float half_span = (MAX_US - MIN_US) * 0.5f; // ≈ 950 us
    float us = NEUTRAL_US + deg * (half_span / 90.0f);

    // Hard clamp to the servo's safe electrical range
    if (us < MIN_US)
        us = MIN_US;
    if (us > MAX_US)
        us = MAX_US;

    return us;
}

void bolt::controller::ServoController::periodElapsed()
{
    GPIOC->BSRR = (GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS2 | GPIO_BSRR_BS3);

    // Convert pulse width (us) to countdown ticks
    for (int i = 0; i < 4; ++i)
    {
        uint32_t us = g_pwm_pulse[i];
        uint32_t ticks = (us + (STEP_US / 2)) / STEP_US; // round
        if (ticks >= FRAME_TICKS)
            ticks = FRAME_TICKS - 1;
        ch_ticks[i] = static_cast<uint16_t>(ticks);
    }

    frame_ticks = FRAME_TICKS;
}

void bolt::controller::ServoController::C_PeriodElapsed(TIM_HandleTypeDef *htim)
{
    auto *self = from(htim);
    if (!self)
        return;

    // New frame?
    if (frame_ticks == 0)
    {
        self->periodElapsed();
    }

    // Decrement per-channel and drop pins low when they expire
    for (int i = 0; i < 4; i++)
    {
        if (ch_ticks[i] > 0)
        {
            ch_ticks[i]--;
            if (ch_ticks[i] == 0)
            {
                switch (i)
                {
                case 0:
                    GPIOC->BSRR = GPIO_BSRR_BR0;
                    break;
                case 1:
                    GPIOC->BSRR = GPIO_BSRR_BR1;
                    break;
                case 2:
                    GPIOC->BSRR = GPIO_BSRR_BR2;
                    break;
                case 3:
                    GPIOC->BSRR = GPIO_BSRR_BR3;
                    break;
                }
            }
        }
    }

    if (frame_ticks > 0)
        frame_ticks--;
}