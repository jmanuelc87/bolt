#include "interface/timer_interface.hpp"

void bolt::timer::PWMSyncTimerPort::setPulses(int16_t pulse1, int16_t pulse2, int16_t pulse3, int16_t pulse4)
{
    htim_->Instance->CCR1 = pulse1;
    htim_->Instance->CCR2 = pulse2;
    htim_->Instance->CCR3 = pulse3;
    htim_->Instance->CCR4 = pulse4;
}

int32_t bolt::timer::CountSyncTimerPort::count()
{
    now_ = __HAL_TIM_GET_COUNTER(htim_);
    int32_t diff = now_ - last_;

    if (diff > ENC_HALF_PERIOD)
        diff -= ENC_PERIOD;
    if (diff < -ENC_HALF_PERIOD)
        diff += ENC_PERIOD;

    last_ = now_;
    return diff;
}

int32_t bolt::timer::CountAsyncTimerPort::count()
{
    return -1;
}