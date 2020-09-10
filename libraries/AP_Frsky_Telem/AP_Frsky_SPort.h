#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 1
#endif

#include <stdint.h>

// for SPort X protocol
#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20
// for SPort D protocol
#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D
// for SPort packet parser
#define TELEMETRY_RX_BUFFER_SIZE    19U  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
#define SPORT_PACKET_SIZE           9U
#define STUFF_MASK                  0x20
#define SPORT_DATA_FRAME            0x10
#define SPORT_UPLINK_FRAME          0x30
#define SPORT_UPLINK_FRAME_RW       0x31
#define SPORT_DOWNLINK_FRAME        0x32

class AP_Frsky_SPort
{
public:
    typedef union {
        struct PACKED {
            uint8_t sensor;
            uint8_t frame;
            uint16_t appid;
            uint32_t data;
        };
        uint8_t raw[8];
    } sport_packet_t;

    // packet parser helpers
    bool process_byte(sport_packet_t &sport_packet, const uint8_t data);
    // sensor id helper
    static uint8_t calc_sport_sensor_id(const uint8_t physical_id);

private:
    enum class ParseState : uint8_t {
        IDLE,
        START,
        IN_FRAME,
        XOR,
    };

    struct {
        uint8_t rx_buffer_count;
        uint8_t rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
        uint8_t last_packet[SPORT_PACKET_SIZE];
        ParseState state;
    } _parse_state;

    bool should_process_packet(const uint8_t *packet, bool discard_duplicates);
    bool get_packet(sport_packet_t &sport_packet, bool discard_duplicates);
};
