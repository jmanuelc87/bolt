#include "controller/motor_controller.hpp"

bool bolt::controller::MotorController::setPulse(int16_t speed)
{
    int16_t pulse = this->ignore_dead_zone(speed);

    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (this->motor_id_)
    {
    case MOTOR_ID_M1:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M1_A = pulse;
            PWM_M1_B = 0;
        }
        else
        {
            PWM_M1_A = 0;
            PWM_M1_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M2:
    {
        pulse = -pulse;
        if (pulse >= 0)
        {
            PWM_M2_A = pulse;
            PWM_M2_B = 0;
        }
        else
        {
            PWM_M2_A = 0;
            PWM_M2_B = -pulse;
        }
        break;
    }

    case MOTOR_ID_M3:
    {
        if (pulse >= 0)
        {
            PWM_M3_A = pulse;
            PWM_M3_B = 0;
        }
        else
        {
            PWM_M3_A = 0;
            PWM_M3_B = -pulse;
        }
        break;
    }
    case MOTOR_ID_M4:
    {
        if (pulse >= 0)
        {
            PWM_M4_A = pulse;
            PWM_M4_B = 0;
        }
        else
        {
            PWM_M4_A = 0;
            PWM_M4_B = -pulse;
        }
        break;
    }

    default:
        break;
    }

    return 1;
}

bool bolt::controller::MotorController::stop(uint8_t brake)
{
    if (brake != 0)
        brake = 1;

    switch (this->motor_id_)
    {
    case MOTOR_ID_M1:
        PWM_M1_A = brake * MOTOR_MAX_PULSE;
        PWM_M1_B = brake * MOTOR_MAX_PULSE;
        break;

    case MOTOR_ID_M2:
        PWM_M2_A = brake * MOTOR_MAX_PULSE;
        PWM_M2_B = brake * MOTOR_MAX_PULSE;
        break;

    case MOTOR_ID_M3:
        PWM_M3_A = brake * MOTOR_MAX_PULSE;
        PWM_M3_B = brake * MOTOR_MAX_PULSE;
        break;

    case MOTOR_ID_M4:
        PWM_M4_A = brake * MOTOR_MAX_PULSE;
        PWM_M4_B = brake * MOTOR_MAX_PULSE;
        break;

    default:
        break;
    }

    return 1;
}

int16_t bolt::controller::MotorController::ignore_dead_zone(int16_t pulse)
{
    if (pulse > 0)
        return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0)
        return pulse - MOTOR_IGNORE_PULSE;

    return pulse;
}