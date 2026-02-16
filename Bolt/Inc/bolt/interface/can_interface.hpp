#ifndef BOLT_CAN_INTERFACE_HPP
#define BOLT_CAN_INTERFACE_HPP

#include "can.h"

#include <functional>
#include <cstdint>

#include "interface.hpp"
#include "registry/handle_registry.hpp"

using bolt::registry::HandleRegistry;

#define RX_BUF_MAX 38
#define TX_TIMEOUT_MS 100

#define CAN_RX_ID_DATA 0x700
#define CAN_TX_ID_DATA 0x702
#define CAN_ID_FC 0x701

#define PCI_SF 0x0u // Single Frame
#define PCI_FF 0x1u // First Frame
#define PCI_CF 0x2u // Consecutive Frame
#define PCI_FC 0x3u // Flow Control

#define FC_CTS 0x0u
#define FC_WAIT 0x1u
#define FC_OVFL 0x2u

namespace bolt
{
    namespace can
    {
        class CanBusAsyncPort : public bolt::CanBus
        {
        public:
            CanBusAsyncPort(CAN_HandleTypeDef *hcan) : hcan_(hcan)
            {
                CAN_FilterTypeDef f;
                f.FilterBank = 0;
                f.FilterMode = CAN_FILTERMODE_IDMASK;
                f.FilterScale = CAN_FILTERSCALE_32BIT;
                f.FilterIdHigh = 0x0000;
                f.FilterIdLow = 0x0000;
                f.FilterMaskIdHigh = 0x0000;
                f.FilterMaskIdLow = 0x0000;
                f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
                f.FilterActivation = ENABLE;
                HAL_CAN_ConfigFilter(hcan_, &f);

                onRxFifo0MsgPending = [this]()
                {
                    CAN_RxHeaderTypeDef rh{};
                    uint8_t data[8];
                    HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rh, data);

                    switch (rh.StdId)
                    {
                    case CAN_RX_ID_DATA:
                        isotpRxOnCan(data, rh.DLC);
                        break;

                    case CAN_ID_FC:
                        isotpRxOnFc(data, rh.DLC);
                        break;

                    default:
                        break;
                    }
                };

                HandleRegistry<CanBusAsyncPort, CAN_HandleTypeDef>::registry().insert({hcan_, this});

                HAL_CAN_Start(hcan_); // must be after filters
                if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
                {
                    Error_Handler();
                }
            }

            ~CanBusAsyncPort()
            {
                HAL_CAN_DeactivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
                HAL_CAN_Stop(hcan_);
                HandleRegistry<CanBusAsyncPort, CAN_HandleTypeDef>::registry().erase(hcan_);
            }

            void isotpSend(const uint8_t *data, uint16_t len);

            void isotpRxOnCan(uint8_t *data, uint8_t dlc);

            void isotpRxOnFc(uint8_t *data, uint8_t dlc);

            void sendMessage(uint32_t id, const uint8_t *data, uint8_t len);

            std::function<void()> onRxFifo0MsgPending;
            std::function<void(uint8_t *, uint8_t)> onMessageRecieved;

            friend class HandleRegistry<CanBusAsyncPort, CAN_HandleTypeDef>;

        private:
            CAN_HandleTypeDef *hcan_;

            TaskHandle_t tx_waiter_ = nullptr;

            void isotpSendFcCts(void);
        };
    }
}

#endif /* BOLT_CAN_INTERFACE_HPP */
