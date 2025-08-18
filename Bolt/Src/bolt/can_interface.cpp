#include "interface/can_interface.hpp"


bool bolt::can::CanBusManager::sendMessage(uint32_t id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header;
    header.StdId = id;
    header.ExtId = 0;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(this->hcan_, &header, data, &mailbox) == HAL_OK;
}

static void bolt::can::CanBusManager::rxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    
}

extern "C" HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanBusManager::rxFifo0MsgPendingCallback(hcan);
}