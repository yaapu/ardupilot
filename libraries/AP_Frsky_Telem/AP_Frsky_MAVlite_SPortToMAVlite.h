#pragma once

#include "AP_Frsky_MAVlite_Message.h"
#include "AP_Frsky_SPort.h"

#include <stdint.h>

class AP_Frsky_MAVlite_SPortToMAVlite {
public:

    bool process(AP_Frsky_MAVlite_Message &rxmsg, const AP_Frsky_SPort::sport_packet_t &packet);

private:

    void reset();

    uint8_t current_rx_seq;
    uint8_t payload_next_byte;

    enum class State : uint8_t {
        IDLE=0,
        ERROR,
        WANT_LEN,
        WANT_MSGID,
        WANT_PAYLOAD,
        WANT_CHECKSUM,
        MESSAGE_RECEIVED,
    };
    State parse_state = State::IDLE;

    AP_Frsky_MAVlite_Message _rxmsg;
    void parse(const uint8_t byte, const uint8_t offset);
};
