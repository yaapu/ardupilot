/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <AP_Math/AP_Math.h>
#include "mavlite.h"
#include <stdint.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

void MAVLITE::mavlite_update_checksum(mavlite_message_t* msg, uint8_t c)
{
    msg->checksum += c; //0-1FF
    msg->checksum += msg->checksum >> 8;
    msg->checksum &= 0xFF;
}

void MAVLITE::mavlite_msg_get_float(mavlite_message_t* msg,float* value, uint8_t offset)
{
    uint8_t* float_bytes = (uint8_t*)value;
    float_bytes[0]=msg->payload[offset+0];
    float_bytes[1]=msg->payload[offset+1];
    float_bytes[2]=msg->payload[offset+2];
    float_bytes[3]=msg->payload[offset+3];
}

void MAVLITE::mavlite_msg_get_uint16(mavlite_message_t* msg,uint16_t* value, uint8_t offset)
{
    uint8_t* bytes = (uint8_t*)value;
    bytes[0]=msg->payload[offset+0];
    bytes[1]=msg->payload[offset+1];
}

void MAVLITE::mavlite_msg_get_uint8(mavlite_message_t* msg,uint8_t* value, uint8_t offset)
{
    uint8_t* bytes = (uint8_t*)value;
    bytes[0]=msg->payload[offset+0];
}

uint8_t MAVLITE::bit8_unpack(uint8_t* value, uint8_t bit_count,uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (unsigned i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    return (*value & mask) >> bit_offset;
}

void MAVLITE::bit8_pack(uint8_t* value, uint8_t bit_value, uint8_t bit_count,uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (unsigned i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    *value |= (bit_value<<bit_offset) & mask;
}

void MAVLITE::mavlite_msg_get_string(mavlite_message_t* msg, char* value, uint8_t offset)
{
    strncpy(value, (char*)msg->payload + offset, MIN((uint8_t)16,msg->len - offset));
    // terminator
    value[MIN((uint8_t)16,msg->len - offset)] = 0x0;
}

void MAVLITE::mavlite_msg_set_float(mavlite_message_t* msg,float* value, uint8_t offset)
{
    uint8_t* float_bytes = (uint8_t*)value;

    msg->payload[offset+0] = float_bytes[0];
    msg->payload[offset+1] = float_bytes[1];
    msg->payload[offset+2] = float_bytes[2];
    msg->payload[offset+3] = float_bytes[3];

    msg->len += 4;
}

void MAVLITE::mavlite_msg_set_uint16(mavlite_message_t* msg,uint16_t* value, uint8_t offset)
{
    uint8_t* bytes = (uint8_t*)value;

    msg->payload[offset+0] = bytes[0];
    msg->payload[offset+1] = bytes[1];

    msg->len += 2;
}

void MAVLITE::mavlite_msg_set_uint8(mavlite_message_t* msg,uint8_t* value, uint8_t offset)
{
    uint8_t* bytes = (uint8_t*)value;

    msg->payload[offset+0] = bytes[0];

    msg->len += 1;
}

void MAVLITE::mavlite_msg_set_string(mavlite_message_t* msg, char* value, uint8_t offset)
{
    strncpy((char*)msg->payload + offset,value, MIN((uint8_t)16,strlen(value)));
    // terminator
    value[MIN((uint8_t)16,strlen(value))] = 0x0;
    msg->len += MIN((uint8_t)16,strlen(value));
}

void MAVLITE::mavlite_init_parse(mavlite_message_t* msg, mavlite_status_t* status)
{
    msg->checksum = 0;
    msg->len = 0;
    msg->msgid = 0;

    status->current_rx_seq = 0;
    status->payload_next_byte = 0;
    status->parse_state = PARSE_STATE_IDLE;
}

void MAVLITE::mavlite_rx_parse(uint8_t byte,uint8_t offset,  mavlite_message_t* rxmsg, mavlite_status_t* status)
{
    switch (status->parse_state) {
        case PARSE_STATE_IDLE:
        case PARSE_STATE_ERROR:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
                mavlite_update_checksum(rxmsg, byte);
            } else {
                status->parse_state = PARSE_STATE_ERROR;
            }
            break;
        case PARSE_STATE_GOT_START:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
            } else {
                rxmsg->len = byte;
                status->parse_state = PARSE_STATE_GOT_LEN;
            }
            mavlite_update_checksum(rxmsg, byte);
            break;
        case PARSE_STATE_GOT_LEN:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
            } else {
                rxmsg->msgid = byte;
                status->parse_state = PARSE_STATE_GOT_MSGID;
            }
            mavlite_update_checksum(rxmsg, byte);
            break;
        case PARSE_STATE_GOT_MSGID:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
            } else {
                rxmsg->payload[status->payload_next_byte++] = byte;
                status->parse_state = PARSE_STATE_GOT_PAYLOAD;
            }
            mavlite_update_checksum(rxmsg, byte);
            break;
        case PARSE_STATE_GOT_SEQ:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
                mavlite_update_checksum(rxmsg, byte);
            } else {
                if ( status->payload_next_byte < rxmsg->len ) {
                    rxmsg->payload[status->payload_next_byte++] = byte;
                    status->parse_state = PARSE_STATE_GOT_PAYLOAD;
                    mavlite_update_checksum(rxmsg, byte);
                } else {
                    if ( rxmsg->checksum == byte ) {
                        status->parse_state = PARSE_STATE_MESSAGE_RECEIVED;
                    } else {
                        status->parse_state = PARSE_STATE_ERROR;
                    }
                }
            }
            break;
        case PARSE_STATE_GOT_PAYLOAD:
            if ( offset == 0) {
                if ( byte == 0x00 ) {
                    mavlite_init_parse(rxmsg, status);
                    status->parse_state = PARSE_STATE_GOT_START;
                } else {
                    if ((byte & 0x3F) != status->current_rx_seq + 1) {
                        status->parse_state = PARSE_STATE_ERROR;
                    } else {
                        status->current_rx_seq = (byte & 0x3F);
                        status->parse_state = PARSE_STATE_GOT_SEQ;
                    }
                }
                mavlite_update_checksum(rxmsg, byte);
            } else {
                if ( status->payload_next_byte < rxmsg->len ) {
                    rxmsg->payload[status->payload_next_byte++] = byte;
                    mavlite_update_checksum(rxmsg, byte);
                } else {
                    if ( rxmsg->checksum == byte ) {
                        status->parse_state = PARSE_STATE_MESSAGE_RECEIVED;
                    } else {
                        status->parse_state = PARSE_STATE_ERROR;
                    }
                }
            }
            break;
        case PARSE_STATE_MESSAGE_RECEIVED:
            if ( offset == 0 && byte == 0x00 ) {
                mavlite_init_parse(rxmsg, status);
                status->parse_state = PARSE_STATE_GOT_START;
                mavlite_update_checksum(rxmsg, byte);
            }
            break;
    }
}

void MAVLITE::mavlite_tx_parse(uint8_t* byte,uint8_t offset,  mavlite_message_t* txmsg, mavlite_status_t* status)
{
    switch (status->parse_state) {
        case PARSE_STATE_IDLE:
        case PARSE_STATE_ERROR:
            if ( offset == 0 ) {
                *byte = 0x00;
                status->parse_state = PARSE_STATE_GOT_START;
                mavlite_update_checksum(txmsg, *byte);
            } else {
                status->parse_state = PARSE_STATE_ERROR;
            }
            break;
        case PARSE_STATE_GOT_START:
            *byte = txmsg->len;
            status->parse_state = PARSE_STATE_GOT_LEN;
            mavlite_update_checksum(txmsg, *byte);
            break;
        case PARSE_STATE_GOT_LEN:
            *byte = txmsg->msgid;
            status->parse_state = PARSE_STATE_GOT_MSGID;
            mavlite_update_checksum(txmsg, *byte);
            break;
        case PARSE_STATE_GOT_MSGID:
            *byte = txmsg->payload[status->payload_next_byte++];
            status->parse_state = PARSE_STATE_GOT_PAYLOAD;
            mavlite_update_checksum(txmsg, *byte);
            break;
        case PARSE_STATE_GOT_SEQ:
            if ( status->payload_next_byte < txmsg->len ) {
                *byte = txmsg->payload[status->payload_next_byte++];
                status->parse_state = PARSE_STATE_GOT_PAYLOAD;
                mavlite_update_checksum(txmsg, *byte);
            } else {
                *byte = txmsg->checksum;
                status->parse_state = PARSE_STATE_MESSAGE_RECEIVED;
            }
            break;
        case PARSE_STATE_GOT_PAYLOAD:
            if ( offset == 0) {
                *byte = ++status->current_rx_seq;
                mavlite_update_checksum(txmsg, *byte);
                status->parse_state = PARSE_STATE_GOT_SEQ;
            } else {
                if ( status->payload_next_byte < txmsg->len ) {
                    *byte = txmsg->payload[status->payload_next_byte++];
                    mavlite_update_checksum(txmsg, *byte);
                } else {
                    *byte = (uint8_t)txmsg->checksum;
                    status->parse_state = PARSE_STATE_MESSAGE_RECEIVED;
                }
            }
            break;
        case PARSE_STATE_MESSAGE_RECEIVED:
            break;
    }
}

#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL