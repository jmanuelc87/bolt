#ifndef BOLT_UTILS_H
#define BOLT_UTILS_H

#include <stdint.h>
#include <stddef.h>

#if defined(__GNUC__)
#define FORCE_INLINE __attribute__((always_inline)) inline
#else
#define FORCE_INLINE inline
#endif

static FORCE_INLINE uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    // CCITT-FALSE polynomial 0x1021, init 0xFFFF
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

#endif /* BOLT_UTILS_H */
