#ifndef BOLT_QUEUES_HPP
#define BOLT_QUEUES_HPP

#include <cstdint>
#include "cmsis_os2.h"
#include "definitions.hpp"

typedef struct
{
    uint16_t size;
    uint8_t data[BUFF_SIZE];
} Message;

extern osMessageQueueId_t processQueue;
extern osMessageQueueId_t queryQueue;

#endif /* BOLT_QUEUES_HPP */
