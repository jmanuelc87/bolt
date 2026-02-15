#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

inline void MX_TIM1_Init(void) {}
inline void MX_TIM2_Init(void) {}
inline void MX_TIM3_Init(void) {}
inline void MX_TIM4_Init(void) {}
inline void MX_TIM5_Init(void) {}
inline void MX_TIM7_Init(void) {}
inline void MX_TIM8_Init(void) {}
inline void HAL_TIM_MspPostInit(TIM_HandleTypeDef *) {}

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
