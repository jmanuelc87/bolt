#ifndef BOLT_UTILS_H
#define BOLT_UTILS_H

#include <stdint.h>
#include <stddef.h>

#include "queues.hpp"
#include "cmsis_os2.h"
#include "stm32f1xx_hal.h"

#if defined(__GNUC__)
#define FORCE_INLINE __attribute__((always_inline)) inline
#else
#define FORCE_INLINE inline
#endif

extern "C" osThreadId_t ledTaskHandle;

static FORCE_INLINE uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static FORCE_INLINE uint16_t u16be(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static FORCE_INLINE uint32_t u32be(const uint8_t *p)
{
    return (uint32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3]);
}
static FORCE_INLINE int16_t i16be(const uint8_t *p) { return (int16_t)u16be(p); }

static size_t build_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out, size_t out_cap)
{
    if (len > MAX_PAYLOAD || out_cap < (size_t)(1 + 1 + 1 + len + 2 + 1))
        return 0;
    size_t i = 0;
    out[i++] = SOF;
    out[i++] = type;
    out[i++] = len;
    for (uint8_t k = 0; k < len; ++k)
        out[i++] = payload[k];
    const uint16_t c = crc16_ccitt(&out[1], (size_t)2 + len);
    out[i++] = (uint8_t)(c >> 8);
    out[i++] = (uint8_t)(c & 0xFF);
    out[i++] = EOF_;
    return i;
}

static FORCE_INLINE void send_message(const char *data)
{
    Message m;

    uint16_t len = build_frame(0x01, reinterpret_cast<const uint8_t *>(data), strlen(data), m.data, sizeof(m.data));
    m.size = len;

    osStatus_t s = osMessageQueuePut(queryQueue, &m, 0, 0);
    if (s != osOK)
    {
        // failed message
    }
}

static FORCE_INLINE void ok_transmission()
{
    osThreadFlagsSet(ledTaskHandle, 0x01);
}

#endif /* BOLT_UTILS_H */
