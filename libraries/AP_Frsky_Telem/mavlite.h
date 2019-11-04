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
#pragma once

#include <stdint.h>
#define MAVLITE_MAX_PAYLOAD_LEN 31 //7 float params + cmd_id + options

namespace MAVLITE {
   typedef struct {
      uint8_t msgid = 0;      // ID of message in payload
      uint8_t len = 0;        // Length of payload
      uint8_t payload[MAVLITE_MAX_PAYLOAD_LEN];
      uint8_t checksum = 0;   // sent at end of packet
   } mavlite_message_t;

   typedef enum : uint8_t {
      PARSE_STATE_IDLE=0,
      PARSE_STATE_ERROR,
      PARSE_STATE_GOT_START,
      PARSE_STATE_GOT_LEN,
      PARSE_STATE_GOT_SEQ,
      PARSE_STATE_GOT_MSGID,
      PARSE_STATE_GOT_PAYLOAD,
      PARSE_STATE_MESSAGE_RECEIVED
   } mavlite_parse_state_t; // state machine for mavlite messages

   typedef struct {
      mavlite_parse_state_t parse_state = PARSE_STATE_IDLE;
      uint8_t current_rx_seq = 0;
      uint8_t payload_next_byte = 0;
   } mavlite_status_t;

   void mavlite_init_parse(mavlite_message_t* msg, mavlite_status_t* status);
   void mavlite_rx_parse(uint8_t byte,uint8_t offset,  mavlite_message_t* rxmsg, mavlite_status_t* status);
   void mavlite_tx_parse(uint8_t* byte,uint8_t offset,  mavlite_message_t* txmsg, mavlite_status_t* status);
   void mavlite_update_checksum(mavlite_message_t* msg, uint8_t c);

   void mavlite_msg_get_float(mavlite_message_t* msg,float* value, uint8_t offset);
   void mavlite_msg_set_float(mavlite_message_t* msg,float* value, uint8_t offset);

   void mavlite_msg_get_string(mavlite_message_t* msg, char* value, uint8_t offset);
   void mavlite_msg_set_string(mavlite_message_t* msg, char* value, uint8_t offset);

   void mavlite_msg_get_uint16(mavlite_message_t* msg,uint16_t* value, uint8_t offset);
   void mavlite_msg_set_uint16(mavlite_message_t* msg,uint16_t* value, uint8_t offset);

   void mavlite_msg_get_uint8(mavlite_message_t* msg,uint8_t* value, uint8_t offset);
   void mavlite_msg_set_uint8(mavlite_message_t* msg,uint8_t* value, uint8_t offset);

   void bit8_pack(uint8_t* value, uint8_t bit_value, uint8_t bit_count,uint8_t bit_offset);
   uint8_t bit8_unpack(uint8_t* value, uint8_t bit_count,uint8_t bit_offset);
}
