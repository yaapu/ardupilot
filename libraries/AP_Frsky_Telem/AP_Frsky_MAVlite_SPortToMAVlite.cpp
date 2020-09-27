#include "AP_Frsky_MAVlite_SPortToMAVlite.h"

#include "AP_Frsky_MAVlite.h"

void AP_Frsky_MAVlite_SPortToMAVlite::reset(void)
{
    _rxmsg.checksum = 0;

    current_rx_seq = 0;
    payload_next_byte = 0;
    parse_state = State::WANT_LEN;
}

/*
 Parses sport packets and if successfull fills the rxmsg mavlite struct
 */
bool AP_Frsky_MAVlite_SPortToMAVlite::process(AP_Frsky_MAVlite_Message &rxmsg, const AP_Frsky_SPort::sport_packet_t &packet)
{
    // the two skipped bytes in packet.raw here are sensor and frame.
    // appid and data are used to transport the mavlite message.

    // if the first byte of any sport packet is zero then we reset:
    if (packet.raw[2] == 0x00) {
        reset();
    } else {
        parse(packet.raw[2], 0);
    }

    for (uint8_t i=3; i<ARRAY_SIZE(packet.raw); i++) {
        parse(packet.raw[i], i-2);
    }
    if (parse_state == State::MESSAGE_RECEIVED) {
        rxmsg = _rxmsg;
        return true;
    }
    return false;
}

void AP_Frsky_MAVlite_SPortToMAVlite::parse(uint8_t byte, uint8_t offset)
{
    switch (parse_state) {

    case State::IDLE:
        // it is an error to receive anything but offset==0 byte=0xx0 in this state
        parse_state = State::ERROR;
        return;

    case State::ERROR:
        // waiting for offset==0 && byte==0x00 to bump us into WANT_LEN
        return;

    case State::WANT_LEN:
        _rxmsg.len = byte;
        _rxmsg.update_checksum(byte);
        parse_state = State::WANT_MSGID;
        return;

    case State::WANT_MSGID:
        _rxmsg.msgid = byte;
        _rxmsg.update_checksum(byte);
        if (_rxmsg.len == 0) {
            parse_state = State::WANT_CHECKSUM;
        } else {
            parse_state = State::WANT_PAYLOAD;
        }
        return;

    case State::WANT_PAYLOAD:
        _rxmsg.update_checksum(byte);
        if (offset == 0) {
            // start of non-initial SPort packet - make sure seqno is correct
            const uint8_t received_seq = (byte & 0x3F);
            if (received_seq != current_rx_seq + 1) {
                // sequence error
                parse_state = State::ERROR;
                return;
            }
            current_rx_seq = received_seq;
            return;
        }

        // add byte to payload
        _rxmsg.payload[payload_next_byte++] = byte;

        if (payload_next_byte >= _rxmsg.len) {
            parse_state = State::WANT_CHECKSUM;
        }
        return;

    case State::WANT_CHECKSUM:
        if (_rxmsg.checksum != byte) {
            gcs().chan(0)->send_text(MAV_SEVERITY_INFO, "checksum error");
            parse_state = State::ERROR;
            return;
        }
        parse_state = State::MESSAGE_RECEIVED;
        return;

    case State::MESSAGE_RECEIVED:
        return;
    }
}
