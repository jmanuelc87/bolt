#ifndef BOLT_CAN_INTERFACE_HPP
#define BOLT_CAN_INTERFACE_HPP

#include "interface.hpp"


namespace bolt
{
    namespace can
    {
        class CanBusManager : public bolt::CanBus
        {
        public:
            CanBusManager(CAN_HandleTypeDef *can) : hcan_(can) {};

            bool sendMessage(uint32_t id, const uint8_t* data, uint8_t len);

            std::function<void(uint32_t, const uint8_t*, uint8_t)> rxCallback;
        private:
            CAN_HandleTypeDef* hcan_;

            void handleRxMessage();

            static void rxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
        };
    }
}

#endif /* BOLT_CAN_INTERFACE_HPP */
