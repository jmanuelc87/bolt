#include "debug.h"


PUTCHAR_PROTOTYPE
{
  #ifdef DEBUG_ENABLED
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  #endif

  return ch;
}

void flush()
{
    HAL_UART_Transmit(&huart1, (uint8_t*)NULL, 0, HAL_MAX_DELAY); 
}