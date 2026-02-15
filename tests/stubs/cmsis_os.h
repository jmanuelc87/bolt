#ifndef CMSIS_OS_STUB_H
#define CMSIS_OS_STUB_H

#include "cmsis_os2.h"

typedef void *TaskHandle_t;

inline void vTaskDelay(uint32_t) {}
#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))
#define configASSERT(x) ((void)(x))

#endif
