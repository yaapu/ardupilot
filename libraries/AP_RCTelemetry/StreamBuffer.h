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

#pragma once

#include <AP_Common/AP_Common.h>

class StreamBuffer {
public:
    // simple buffer-based serializer/deserializer without implicit size check

    StreamBuffer(uint8_t *ptr, uint8_t *end);

    void write_uint8(uint8_t val) { *ptr++ = val; }
    void write_uint16(uint16_t val);
    void write_uint32(uint32_t val);
    void write_uint16_big_endian(uint16_t val);
    void write_uint32_big_endian(uint32_t val);
    void fill(uint8_t data, int len);
    void write_data(const void *data, int len);
    void write_string(const char *string);
    void write_string_with_zero_terminator(const char *string);

    uint8_t read_uint8() { return *ptr++; }
    uint16_t read_uint16();
    uint32_t read_uint32();
    void read_data(void *data, int len);

    int32_t remaining();
    uint8_t* data();
    void advance(uint32_t size);
    void switch_to_reader(uint8_t * base);

    // crc functions
    uint8_t crc8_high_first(uint8_t *ptr, uint8_t len);

private:
    uint8_t *ptr;          // data pointer must be first (StreamBuffer* is equivalent to uint8_t **)
    uint8_t *end_ptr;
};
