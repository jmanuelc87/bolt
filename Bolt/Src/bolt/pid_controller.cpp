#include "controller/pid_controller.hpp"

namespace bolt
{
    namespace controller
    {
        float PIDController::compute(float setpoint, float measurement)
        {
            float error = setpoint - measurement;

            integral_ += error * dt_;

            float derivative = (error - prevError_) / dt_;

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

            output = clamp(output);

            prevError_ = error;

            return output;
        }

        void PIDController::reset()
        {
            integral_ = 0.0f;
            prevError_ = 0.0f;
        }

        void PIDController::setGains(float kp, float ki, float kd)
        {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
        }

        void PIDController::setOutputLimits(float min, float max)
        {
            outMin_ = min;
            outMax_ = max;
        }

        void PIDController::setSetpoint(float setpoint)
        {
            setpoint_ = setpoint;
        }

        float PIDController::clamp(float value)
        {
            if (value > outMax_)
                return outMax_;
            if (value < outMin_)
                return outMin_;
            return value;
        }
    }
}