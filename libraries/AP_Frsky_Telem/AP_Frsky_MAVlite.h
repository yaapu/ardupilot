#pragma once

/*

  Wire Protocol:

  Several SPort packets make up a MAVlite message.

  A maximum of six relevant data bytes are present in the SPort packet.

  If the first byte of any SPort message is 0x00 then the parser is reset.

  The first sport packet contains len (at offset 0), msgid then payload bytes.

  Subsequent SPort packets contain a sequence number (starting at 1), followed by more payload bytes.

  When sufficient payload bytes have been received (based on "len"), a single checksum byte arrives.

*/


#define MAVLITE_MAX_PAYLOAD_LEN                 31 // 7 float params + cmd_id + options
#define MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)    static_cast<uint8_t>(1 + ceilf((LEN-2)/5.0f)) // number of sport packets required to transport a message with LEN payload
#define SPORT_PACKET_QUEUE_LENGTH               static_cast<uint8_t>(30U*MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN))
