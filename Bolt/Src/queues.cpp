#include "queues.hpp"

osMessageQueueId_t processQueue;
osMessageQueueId_t queryQueue;

extern "C" void AppQueuesInit(void)
{
    processQueue = osMessageQueueNew(/*queue length*/ 8, /*item size*/ sizeof(Message), NULL);
    queryQueue = osMessageQueueNew(/*queue length*/ 8, /*item size*/ sizeof(Message), NULL);
}