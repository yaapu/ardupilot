#include "AP_Frsky_MAVlite_MAVliteToSPort.h"

#include "AP_Frsky_MAVlite.h"

#include "AP_Frsky_SPort.h"

void AP_Frsky_MAVlite_MAVliteToSPort::reset(AP_Frsky_MAVlite_Message &txmsg)
{
    txmsg.checksum = 0;

    current_rx_seq = 0;
    payload_next_byte = 0;
    parse_state = State::IDLE;
}

/*
 Warning:
 make sure that all packets pushed by this method are sequential and not interleaved by packets inserted by another thread!
 */
bool AP_Frsky_MAVlite_MAVliteToSPort::process(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue, AP_Frsky_MAVlite_Message &msg)
{
    // let's check if there's enough room to send it
    if (queue.space() < MAVLITE_MSG_SPORT_PACKETS_COUNT(msg.len)) {
        return false;
    }
    reset(msg);
    // prevent looping forever
    uint8_t packet_count = 0;
    while (parse_state != State::MESSAGE_RECEIVED && packet_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {

        AP_Frsky_SPort::sport_packet_t packet {};

        for (uint8_t i=0; i<6; i++) {
            // read msg and fill packet one byte at the time, ignore sensor and frame byte
            encode(packet.raw[i+2], i, msg);
        }

        if (parse_state == State::ERROR) {
            break;
        }

        queue.push(packet);
    }
    parse_state = State::IDLE;
    return true;
}

bool AP_Frsky_MAVlite_MAVliteToSPort::encode(uint8_t &byte, const uint8_t offset, AP_Frsky_MAVlite_Message &txmsg)
{
    switch (parse_state) {
    case State::IDLE:
    case State::ERROR:
        if (offset == 0) {
            byte = 0x00;
            reset(txmsg);
            parse_state = State::GOT_START;
        } else {
            parse_state = State::ERROR;
        }
        break;
    case State::GOT_START:
        byte = txmsg.len;
        parse_state = State::GOT_LEN;
        txmsg.update_checksum(byte);
        break;
    case State::GOT_LEN:
        byte = txmsg.msgid;
        parse_state = State::GOT_MSGID;
        txmsg.update_checksum(byte);
        break;
    case State::GOT_MSGID:
        byte = txmsg.payload[payload_next_byte++];
        parse_state = State::GOT_PAYLOAD;
        txmsg.update_checksum(byte);
        break;
    case State::GOT_SEQ:
        if (payload_next_byte < txmsg.len) {
            byte = txmsg.payload[payload_next_byte++];
            parse_state = State::GOT_PAYLOAD;
            txmsg.update_checksum(byte);
        } else {
            byte = txmsg.checksum;
            parse_state = State::MESSAGE_RECEIVED;
            return true;
        }
        break;
    case State::GOT_PAYLOAD:
        if (offset == 0) {
            byte = ++current_rx_seq;
            txmsg.update_checksum(byte);
            parse_state = State::GOT_SEQ;
        } else {
            if (payload_next_byte < txmsg.len) {
                byte = txmsg.payload[payload_next_byte++];
                txmsg.update_checksum(byte);
            } else {
                byte = (uint8_t)txmsg.checksum;
                parse_state = State::MESSAGE_RECEIVED;
                return true;
            }
        }
        break;
    case State::MESSAGE_RECEIVED:
        break;
    }
    return false;
}
