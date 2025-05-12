#ifndef BOLT_MOTOR_COMMAND_H
#define BOLT_MOTOR_COMMAND_H

#include "command.hpp"

namespace bolt
{
    namespace motor
    {
        class SetPwmMotorDutyCycleCommand : public bolt::Command
        {
        public:
            SetPwmMotorDutyCycleCommand(int motorId, float dutyCycle) : motorId_(motorId), dutyCycle_(dutyCycle) {};
            void execute() override;

        private:
            int motorId_;
            float dutyCycle_;
        };
    }
}

#endif /* BOLT_MOTOR_COMMAND_H */
