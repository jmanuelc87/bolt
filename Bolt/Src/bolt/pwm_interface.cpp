#include "interface/pwm_interface.hpp"

bool bolt::pwm::PWMTimerInterface::setPulse(int16_t speed)
{
    int16_t pulse = this->ignore_dead_zone(speed);

    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (this->motorId_)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            this->tim_->CCR1 = pulse;
            this->tim_->CCR2 = 0;
        }
        else
        {
            this->tim_->CCR1 = 0;
            this->tim_->CCR2 = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            this->tim_->CCR3 = pulse;
            this->tim_->CCR4 = 0;
        }
        else
        {
            this->tim_->CCR3 = 0;
            this->tim_->CCR4 = -pulse;
        }
        break;
    }

    case MOTOR_ID_M3:
    {
        if (pulse >= 0)
        {
            this->tim_->CCR4 = pulse;
            this->tim_->CCR1 = 0;
        }
        else
        {
            this->tim_->CCR4 = 0;
            this->tim_->CCR1 = -pulse;
        }
        break;
    }
    case MOTOR_ID_M4:
    {
        if (pulse >= 0)
        {
            this->tim_->CCR2 = pulse;
            this->tim_->CCR3 = 0;
        }
        else
        {
            this->tim_->CCR2 = 0;
            this->tim_->CCR3 = -pulse;
        }
        break;
    }

    default:
        break;
    }

    return 1;
}

int16_t bolt::pwm::PWMTimerInterface::ignore_dead_zone(int16_t pulse)
{
    if (pulse > 0)
        return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0)
        return pulse - MOTOR_IGNORE_PULSE;

    return pulse;
}