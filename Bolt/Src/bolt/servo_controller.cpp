#include "controller/servo_controller.hpp"

bool bolt::controller::ServoController::setPulse(int16_t pulse)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        this->g_num_angle[i] = pulse;
    }

    return 1;
}

bool bolt::controller::ServoController::setAngle(int16_t angle, uint8_t servo_id)
{
    this->g_num_angle[servo_id] = this->PwmServo_Angle_To_Pulse(angle);
    return 1;
}

uint16_t bolt::controller::ServoController::PwmServo_Angle_To_Pulse(uint8_t angle)
{
    if (angle > 180)
        angle = 180;

    uint16_t pulse = angle * (2000 / 180);
    return pulse;
}

void bolt::controller::ServoController::periodElapsed()
{
    g_pwm_pulse++;

    if (g_pwm_pulse <= g_num_angle[0])
        this->servo1_.setHigh();
    else
        this->servo1_.setLow();

    if (g_pwm_pulse <= g_num_angle[1])
        this->servo2_.setHigh();
    else
        this->servo2_.setLow();

    if (g_pwm_pulse <= g_num_angle[2])
        this->servo3_.setHigh();
    else
        this->servo3_.setLow();

    if (g_pwm_pulse <= g_num_angle[3])
        this->servo4_.setHigh();
    else
        this->servo4_.setLow();

    if (g_pwm_pulse >= 2000)
        g_pwm_pulse = 0;
}

void bolt::controller::ServoController::C_PeriodElapsed(TIM_HandleTypeDef *htim)
{
    auto *self = from(htim);
    if (self)
    {
        self->periodElapsed();
    }
}