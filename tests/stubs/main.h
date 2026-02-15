#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

inline void Error_Handler(void) {}
inline void MX_FREERTOS_Init(void) {}

// Stub GPIO port instances
extern GPIO_TypeDef gStubGPIOB, gStubGPIOC, gStubGPIOD;

#define GPIOB (&gStubGPIOB)
#define GPIOC (&gStubGPIOC)
#define GPIOD (&gStubGPIOD)

#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)

#define ENABLE 1

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOC
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOC
#define S4_Pin GPIO_PIN_3
#define S4_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_5
#define BEEP_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define ICM20948_CS_Pin GPIO_PIN_12
#define ICM20948_CS_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOD

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
