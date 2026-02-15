#ifndef BOLT_PID_CONTROLLER_HPP
#define BOLT_PID_CONTROLLER_HPP

#include <cstdint>
#include <functional>

#include "interface/timer_interface.hpp"

using bolt::timer::ProcessAsyncTimerPort;

namespace bolt
{
    namespace controller
    {
        class PIDController
        {
        public:
            PIDController(ProcessAsyncTimerPort *sampler,
                          float kp, float ki, float kd, float dt,
                          float outMin, float outMax)
                : sampler_(sampler), kp_(kp), ki_(ki), kd_(kd), dt_(dt),
                  outMin_(outMin), outMax_(outMax)
            {
                sampler_->timElapsedCompleteCallback = [this]()
                {
                    if (measureCallback && outputCallback)
                    {
                        float measurement = measureCallback();
                        float output = compute(setpoint_, measurement);
                        outputCallback(output);
                    }
                };
            }

            ~PIDController() {}

            float compute(float setpoint, float measurement);
            void reset();
            void setGains(float kp, float ki, float kd);
            void setOutputLimits(float min, float max);
            void setSetpoint(float setpoint);

            std::function<float()> measureCallback;
            std::function<void(float)> outputCallback;

        private:
            ProcessAsyncTimerPort *sampler_;

            float kp_;
            float ki_;
            float kd_;
            float dt_;

            float outMin_;
            float outMax_;

            float setpoint_ = 0.0f;
            float integral_ = 0.0f;
            float prevError_ = 0.0f;

            float clamp(float value);
        };
    }
}

#endif /* BOLT_PID_CONTROLLER_HPP */
