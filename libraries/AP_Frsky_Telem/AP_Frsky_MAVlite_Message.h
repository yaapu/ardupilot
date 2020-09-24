#pragma once

#include "AP_Frsky_MAVlite.h"

#include <stdint.h>

class AP_Frsky_MAVlite_Message {
public:
    // helpers
    bool get_float(float &value, const uint8_t offset) const {
        return get_bytes((uint8_t*)&value, offset, 4);
    }
    bool set_float(const float value, const uint8_t offset) {
        return set_bytes((uint8_t*)&value, offset, 4);
    }

    bool get_string(char* value, const uint8_t offset) const;
    bool set_string(const char* value, const uint8_t offset);

    bool get_uint16(uint16_t &value, const uint8_t offset) const {
        return get_bytes((uint8_t*)&value, offset, 2);
    }
    bool set_uint16(const uint16_t value, const uint8_t offset) {
        return set_bytes((uint8_t*)&value, offset, 2);
    }

    bool get_uint8(uint8_t &value, const uint8_t offset) const {
        return get_bytes((uint8_t*)&value, offset, 1);;
    }
    bool set_uint8(const uint8_t value, const uint8_t offset) {
        return set_bytes((uint8_t*)&value, offset, 1);
    }

    uint8_t msgid = 0;                          // ID of message in payload
    uint8_t len = 0;                            // Length of payload
    uint8_t payload[MAVLITE_MAX_PAYLOAD_LEN];
    int16_t checksum = 0;                       // sent at end of packet

    void update_checksum(const uint8_t c);

    static void bit8_pack(uint8_t &value, const uint8_t bit_value, const uint8_t bit_count, const uint8_t bit_offset);
    static uint8_t bit8_unpack(const uint8_t value, const  uint8_t bit_count, const uint8_t bit_offset);

private:
    bool get_bytes(uint8_t *bytes, const uint8_t offset, const uint8_t count) const;
    bool set_bytes(const uint8_t *bytes,  const uint8_t offset, const uint8_t count);
};
