#include "interface/timer_interface.hpp"

void bolt::timer::PWMSyncTimerPort::setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4)
{
    htim_->Instance->CCR1 = pulse1;
    htim_->Instance->CCR2 = pulse2;
    htim_->Instance->CCR3 = pulse3;
    htim_->Instance->CCR4 = pulse4;
}

uint32_t bolt::timer::CountSyncTimerPort::getCount()
{
    now_ = __HAL_TIM_GET_COUNTER(htim_);
    int64_t diff = (int64_t)now_ - (int64_t)last_;

    if (diff > (int64_t)ENC_HALF)
        diff -= (int64_t)ENC_PERIOD;
    if (diff < -(int64_t)ENC_HALF)
        diff += (int64_t)ENC_PERIOD;

    last_ = now_;
    return (uint32_t)diff;
}
