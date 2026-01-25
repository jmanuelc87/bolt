#include "registry/handle_registry.hpp"

#include "interface/can_interface.hpp"
#include "interface/timer_interface.hpp"

using bolt::can::CanBusAsyncPort;
using bolt::timer::ProcessAsyncTimerPort;
using bolt::timer::PROC_HandleTypeDef;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    if (auto *inst = bolt::registry::HandleRegistry<CanBusAsyncPort, CAN_HandleTypeDef>::from(h); inst && inst->onRxFifo0MsgPending)
    {
        inst->onRxFifo0MsgPending();
    }
}

extern "C" void v2Process_Task(void *argument)
{
    (void)argument;

    while (true)
    {
        std::unordered_map registry = bolt::registry::HandleRegistry<ProcessAsyncTimerPort, PROC_HandleTypeDef>::registry();

        for (const auto &[handler, controller] : registry)
        {
            if (++(handler->counter) >= handler->time)
            {
                if (controller && controller->timElapsedCompleteCallback)
                {
                    controller->timElapsedCompleteCallback();
                }

                handler->counter = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
