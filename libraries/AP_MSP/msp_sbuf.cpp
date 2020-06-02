#include <string.h>

#include "msp_sbuf.h"

#if HAL_MSP_ENABLED

uint8_t* MSP::sbuf_ptr(sbuf_t *buf)
{
    return buf->ptr;
}

int MSP::sbuf_bytes_remaining(const sbuf_t *buf)
{
    return buf->end - buf->ptr;
}

void MSP::sbuf_switch_to_reader(sbuf_t *buf, uint8_t *base)
{
    buf->end = buf->ptr;
    buf->ptr = base;
}

void MSP::sbuf_write_u8(sbuf_t *dst, uint8_t val)
{
    *dst->ptr++ = val;
}

void MSP::sbuf_write_u16(sbuf_t *dst, uint16_t val)
{
    sbuf_write_u8(dst, val >> 0);
    sbuf_write_u8(dst, val >> 8);
}

void MSP::sbuf_write_u32(sbuf_t *dst, uint32_t val)
{
    sbuf_write_u8(dst, val >> 0);
    sbuf_write_u8(dst, val >> 8);
    sbuf_write_u8(dst, val >> 16);
    sbuf_write_u8(dst, val >> 24);
}

void MSP::sbuf_write_data(sbuf_t *dst, const void *data, int len)
{
    memcpy(dst->ptr, data, len);
    dst->ptr += len;
}

#endif  //HAL_MSP_ENABLED