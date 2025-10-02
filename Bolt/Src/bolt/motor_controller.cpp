#include "controller/motor_controller.hpp"

void bolt::controller::MotorController::setSpeed(uint8_t motor_id, int16_t pulse)
{
    pulse = ignore_dead_zone(pulse);
    pulse = clamp(pulse);

    if (motor_id >= 4)
        return;

    const auto &pair = motor2ports_[motor_id];

    pulse = -pulse;
    if (pulse >= 0)
    {
        pulse_motor_[pair.first] = pulse;
        pulse_motor_[pair.second] = 0;
    }
    else
    {
        pulse_motor_[pair.first] = 0;
        pulse_motor_[pair.second] = -pulse;
    }

    port1_->setPulses(pulse_motor_[0], pulse_motor_[1], pulse_motor_[2], pulse_motor_[3]);
    port2_->setPulses(pulse_motor_[4], pulse_motor_[5], pulse_motor_[6], pulse_motor_[7]);
}

int16_t bolt::controller::MotorController::ignore_dead_zone(int16_t pulse)
{
    if (pulse > 0)
        return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0)
        return pulse - MOTOR_IGNORE_PULSE;

    return pulse;
}

int16_t bolt::controller::MotorController::clamp(int16_t pulse)
{
    if (pulse > MAX_MOTOR_PULSE)
        pulse = MAX_MOTOR_PULSE;
    if (pulse <= -MAX_MOTOR_PULSE)
        pulse = -MAX_MOTOR_PULSE;

    return pulse;
}