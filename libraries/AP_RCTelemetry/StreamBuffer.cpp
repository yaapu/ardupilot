/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "StreamBuffer.h"

StreamBuffer::StreamBuffer(uint8_t *p, uint8_t *end)
{
    ptr = p;
    end_ptr = end;
}

void StreamBuffer::write_uint16(uint16_t val)
{
    write_uint8(val >> 0);
    write_uint8(val >> 8);
}

void StreamBuffer::write_uint32(uint32_t val)
{
    write_uint8(val >> 0);
    write_uint8(val >> 8);
    write_uint8(val >> 16);
    write_uint8(val >> 24);
}

void StreamBuffer::write_uint16_big_endian(uint16_t val)
{
    write_uint8(val >> 8);
    write_uint8((uint8_t)val);
}

void StreamBuffer::write_uint32_big_endian(uint32_t val)
{
    write_uint8(val >> 24);
    write_uint8(val >> 16);
    write_uint8(val >> 8);
    write_uint8((uint8_t)val);
}

void StreamBuffer::fill(uint8_t d, int len)
{
    memset(ptr, d, len);
    ptr += len;
}

void StreamBuffer::write_data(const void *d, int len)
{
    memcpy(ptr, d, len);
    ptr += len;
}

void StreamBuffer::write_string(const char *string)
{
    write_data(string, strlen(string));
}

void StreamBuffer::write_string_with_zero_terminator(const char *string)
{
    write_data(string, strlen(string) + 1);
}

uint16_t StreamBuffer::read_uint16()
{
    uint16_t ret;
    ret = read_uint8();
    ret |= read_uint8() << 8;
    return ret;
}

uint32_t StreamBuffer::read_uint32()
{
    uint32_t ret;
    ret = read_uint8();
    ret |= read_uint8() <<  8;
    ret |= read_uint8() << 16;
    ret |= read_uint8() << 24;
    return ret;
}

void StreamBuffer::read_data(void *d, int len)
{
    memcpy(d, ptr, len);
}

// reader - return bytes remaining in buffer
// writer - return available space
int32_t StreamBuffer::remaining()
{
    return end_ptr - ptr;
}

uint8_t* StreamBuffer::data()
{
    return ptr;
}

// advance buffer pointer
// reader - skip data
// writer - commit written data
void StreamBuffer::advance(uint32_t size)
{
    ptr += size;
}

// modifies streambuf so that written data are prepared for reading
void StreamBuffer::switch_to_reader(uint8_t *base)
{
    end_ptr = ptr;
    ptr = base;
}

uint8_t StreamBuffer::crc8_high_first(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *p++;
        for (uint8_t i = 8; i > 0; --i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return (crc);
}
