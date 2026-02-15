#ifndef STM32F1XX_HAL_STUB_H
#define STM32F1XX_HAL_STUB_H

#include <cstdint>

#define __IO volatile

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_UNLOCKED = 0, HAL_LOCKED } HAL_LockTypeDef;

// --- GPIO ---
typedef struct { uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

inline void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState) {}
inline void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t) {}

// --- TIM ---
typedef struct
{
    uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2,
        CCER, CNT, PSC, ARR, RCR,
        CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR;
} TIM_TypeDef;

typedef struct __TIM_HandleTypeDef
{
    TIM_TypeDef *Instance;
    HAL_LockTypeDef Lock;
    void (*PeriodElapsedCallback)(struct __TIM_HandleTypeDef *);
} TIM_HandleTypeDef;

typedef enum { HAL_TIM_PERIOD_ELAPSED_CB_ID = 0x0EU } HAL_TIM_CallbackIDTypeDef;

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU

#define __HAL_TIM_SET_COUNTER(__H__, __V__) ((__H__)->Instance->CNT = (__V__))
#define __HAL_TIM_GET_COUNTER(__H__) ((__H__)->Instance->CNT)

inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_RegisterCallback(TIM_HandleTypeDef *, HAL_TIM_CallbackIDTypeDef, void (*)(TIM_HandleTypeDef *)) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_UnRegisterCallback(TIM_HandleTypeDef *, HAL_TIM_CallbackIDTypeDef) { return HAL_OK; }

// Stub TIM peripheral instances (motor_controller.hpp accesses TIM1->CCR1 etc.)
extern TIM_TypeDef gStubTIM1, gStubTIM8;
#define TIM1 (&gStubTIM1)
#define TIM8 (&gStubTIM8)

// --- UART ---
typedef struct __UART_HandleTypeDef
{
    void *Instance;
    HAL_LockTypeDef Lock;
    uint32_t RxXferSize;
    uint32_t RxXferCount;
    uint8_t *pRxBuffPtr;
    void (*TxCpltCallback)(struct __UART_HandleTypeDef *);
    void (*RxCpltCallback)(struct __UART_HandleTypeDef *);
    void (*RxEventCallback)(struct __UART_HandleTypeDef *, uint16_t);
} UART_HandleTypeDef;

typedef enum { HAL_UART_TX_COMPLETE_CB_ID = 0x01U, HAL_UART_RX_COMPLETE_CB_ID = 0x03U } HAL_UART_CallbackIDTypeDef;
typedef void (*pUART_RxEventCallbackTypeDef)(UART_HandleTypeDef *, uint16_t);

inline HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef *, HAL_UART_CallbackIDTypeDef, void (*)(UART_HandleTypeDef *)) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_UnRegisterCallback(UART_HandleTypeDef *, HAL_UART_CallbackIDTypeDef) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_RegisterRxEventCallback(UART_HandleTypeDef *, pUART_RxEventCallbackTypeDef) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_UART_UnRegisterRxEventCallback(UART_HandleTypeDef *) { return HAL_OK; }

// --- SPI ---
typedef struct __SPI_HandleTypeDef
{
    void *Instance;
    HAL_LockTypeDef Lock;
    uint32_t ErrorCode;
} SPI_HandleTypeDef;

inline HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *, uint8_t *, uint16_t, uint32_t) { return HAL_OK; }

// --- CAN ---
typedef struct { uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment, FilterBank, FilterMode, FilterScale, FilterActivation, SlaveStartFilterBank; } CAN_FilterTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, Timestamp, FilterMatchIndex; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct __CAN_HandleTypeDef
{
    void *Instance;
    uint32_t ErrorCode;
} CAN_HandleTypeDef;

#define CAN_FILTERMODE_IDMASK 0x00000000U
#define CAN_FILTERSCALE_32BIT 0x00000001U
#define CAN_FILTER_FIFO0 0x00000000U
#define CAN_RX_FIFO0 0x00000000U
#define CAN_IT_RX_FIFO0_MSG_PENDING 0x00000001U
#define CAN_ID_STD 0x00000000U
#define CAN_RTR_DATA 0x00000000U

inline HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, const CAN_FilterTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, const CAN_TxHeaderTypeDef *, const uint8_t *, uint32_t *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *) { return HAL_OK; }
inline uint32_t HAL_GetTick() { return 0; }

#endif
