#ifndef BOLT_UART_HANDLE_REGISTRY_HPP
#define BOLT_UART_HANDLE_REGISTRY_HPP

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include <unordered_map>

namespace bolt
{
    namespace serial
    {
        template <typename Controller, typename HandleType>
        class UartHandleRegistry
        {
        public:
            static std::unordered_map<HandleType *, Controller *> &registry()
            {
                static std::unordered_map<HandleType *, Controller *> reg;
                return reg;
            }

            static Controller *from(HandleType *handle)
            {
                auto &reg = registry();
                auto it = reg.find(handle);
                return (it == reg.end()) ? nullptr : it->second;
            }

            static void registerCallbacks(HandleType *h)
            {
                if constexpr (std::is_same_v<HandleType, UART_HandleTypeDef>)
                {
                    HAL_StatusTypeDef st;
                    st = HAL_UART_RegisterCallback(h, HAL_UART_TX_COMPLETE_CB_ID, &UartHandleRegistry::C_TxCplt);
                    configASSERT(st == HAL_OK);
                    st = HAL_UART_RegisterRxEventCallback(h, &UartHandleRegistry::C_RxCplt);
                    configASSERT(st == HAL_OK);
                }
                else if constexpr (std::is_same_v<HandleType, TIM_HandleTypeDef>)
                {
                    HAL_StatusTypeDef st;
                    st = HAL_TIM_RegisterCallback(h, HAL_TIM_PERIOD_ELAPSED_CB_ID, &UartHandleRegistry::C_TimElapsedCplt);
                    configASSERT(st == HAL_OK);
                    // Add more TIM event callback registrations if needed
                }
            }

            static void unregisterCallbacks(HandleType *h)
            {
                if constexpr (std::is_same_v<HandleType, UART_HandleTypeDef>)
                {
                    HAL_UART_UnRegisterCallback(h, HAL_UART_TX_COMPLETE_CB_ID);
                    HAL_UART_UnRegisterRxEventCallback(h);
                }
                else if constexpr (std::is_same_v<HandleType, TIM_HandleTypeDef>)
                {
                    HAL_TIM_UnRegisterCallback(h, HAL_TIM_PERIOD_ELAPSED_CB_ID);
                }
            }

        private:
            static void C_TxCplt(HandleType *h)
            {
                if (auto *inst = from(h); inst && inst->txCompleteCallback)
                {
                    inst->txCompleteCallback();
                }
            }
            static void C_RxCplt(HandleType *h, uint16_t Size)
            {
                if (auto *inst = from(h); inst && inst->rxEventCallback)
                {
                    inst->rxEventCallback(Size);
                }
            }

            static void C_TimElapsedCplt(TIM_HandleTypeDef *h)
            {
                if (auto *inst = from(h); inst && inst->timElapsedCompleteCallback)
                {
                    inst->timElapsedCompleteCallback();
                }
            }
        };
    }
}

#endif /* BOLT_UART_HANDLE_REGISTRY_HPP */
