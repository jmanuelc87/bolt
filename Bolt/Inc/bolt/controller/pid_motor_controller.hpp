#ifndef BOLT_PID_MOTOR_CONTROLLER_HPP
#define BOLT_PID_MOTOR_CONTROLLER_HPP

#include "controller/pid_controller.hpp"
#include "controller/motor_controller.hpp"
#include "controller/encoder_controller.hpp"

namespace bolt
{
    namespace controller
    {
        class PIDMotorController : public PIDController
        {
        public:
            PIDMotorController(ProcessAsyncTimerPort *sampler,
                               float kp, float ki, float kd, float dt,
                               float outMin, float outMax,
                               MotorController *motor,
                               EncoderController *encoder,
                               uint8_t motorId)
                : PIDController(sampler, kp, ki, kd, dt, outMin, outMax),
                  motor_(motor), encoder_(encoder), motorId_(motorId)
            {
                measureCallback = [this]() -> float
                {
                    return encoder_->getRPM(motorId_);
                };

#ifdef USE_PID
                outputCallback = [this](float output)
                {
                    motor_->setSpeed(motorId_ - 1, static_cast<int16_t>(output));
                };
#endif
            }

            ~PIDMotorController() {}

            void setTargetRPM(float rpm) { setSetpoint(rpm); }

            void stop(uint8_t brake)
            {
                setSetpoint(0.0f);
                reset();
                motor_->stop(motorId_ - 1, brake);
            }

        private:
            MotorController *motor_;
            EncoderController *encoder_;
            uint8_t motorId_;
        };
    }
}

#endif /* BOLT_PID_MOTOR_CONTROLLER_HPP */
