#pragma once

#include "AP_Frsky_MAVlite_Message.h"
#include "AP_Frsky_SPort.h"

#include <AP_HAL/utility/RingBuffer.h>

#include <stdint.h>

class AP_Frsky_MAVlite_MAVliteToSPort {
public:

    // insert sport packets calculated from mavlite msg into queue
    bool process(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue,
                 const AP_Frsky_MAVlite_Message &msg);

private:

    enum class State : uint8_t {
        WANT_LEN,
        WANT_MSGID,
        WANT_PAYLOAD,
        WANT_CHECKSUM,
        DONE,
    };
    State state = State::WANT_LEN;

    void reset();

    void process_byte(uint8_t byte, ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue);

    AP_Frsky_SPort::sport_packet_t packet {};
    uint8_t packet_offs = 0;

    uint8_t next_seq = 0;
    uint8_t payload_count = 0;
    uint8_t payload_len;

    int16_t checksum;                       // sent at end of packet
    void update_checksum(const uint8_t c);
};
