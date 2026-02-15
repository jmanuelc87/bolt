#include "registry/handle_registry.hpp"

#include "interface/can_interface.hpp"
#include "interface/timer_interface.hpp"

using bolt::can::CanBusAsyncPort;
using bolt::timer::PROC_HandleTypeDef;
using bolt::timer::ProcessAsyncTimerPort;

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

        for (auto &[handler, controller] : registry)
        {
            if (!(--handler->counter))
            {
                if (controller && controller->timElapsedCompleteCallback)
                {
                    controller->timElapsedCompleteCallback();
                }

                handler->counter = handler->timer;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
