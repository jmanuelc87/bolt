#ifndef CMSIS_OS2_STUB_H
#define CMSIS_OS2_STUB_H

#include <cstdint>
#include <cstring>

typedef void *osMessageQueueId_t;
typedef void *osThreadId_t;

// ---------------------------------------------------------------------------
// Captured queue message — tests may inspect this after calling visitor methods
// that queue response frames via send_payload / osMessageQueuePut.
//
// BUFF_SIZE is 38 (definitions.hpp), but the full Message struct is
//   { uint16_t size; uint8_t data[38]; }  = 40 bytes.
// We capture up to 64 bytes raw so we can overlay any struct on top.
// ---------------------------------------------------------------------------
struct CapturedQueueMessage
{
    uint8_t  raw[64];
    unsigned bytes_written;  // how many bytes were memcpy'd (sizeof(Message))
    bool     captured;
};

inline CapturedQueueMessage g_lastQueueMessage = {};

inline uint32_t osMessageQueuePut(osMessageQueueId_t, const void *msg, uint8_t, uint32_t)
{
    if (msg)
    {
        // Message is { uint16_t size; uint8_t data[38]; } = 40 bytes
        constexpr unsigned kMessageSize = 2u + 38u;
        memcpy(g_lastQueueMessage.raw, msg, kMessageSize);
        g_lastQueueMessage.bytes_written = kMessageSize;
        g_lastQueueMessage.captured = true;
    }
    return 0;
}

#endif
