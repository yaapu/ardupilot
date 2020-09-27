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

    uint8_t current_rx_seq = 0;
    uint8_t payload_next_byte = 0;

    enum class State : uint8_t {
        IDLE=0,
        ERROR,
        GOT_START,
        GOT_LEN,
        GOT_SEQ,
        GOT_MSGID,
        GOT_PAYLOAD,
        MESSAGE_RECEIVED,
    };
    State parse_state = State::IDLE;

    void reset();
    bool encode(uint8_t &byte, uint8_t offset, const AP_Frsky_MAVlite_Message &txmsg);

    int16_t checksum;                       // sent at end of packet
    void update_checksum(const uint8_t c);
};
