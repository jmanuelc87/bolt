#include "registry/handle_registry.hpp"

#include "interface/can_interface.hpp"

using bolt::can::CanBusAsyncPort;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    if (auto *inst = bolt::registry::HandleRegistry<CanBusAsyncPort, CAN_HandleTypeDef>::from(h); inst && inst->onRxFifo0MsgPending)
    {
        inst->onRxFifo0MsgPending();
    }
}