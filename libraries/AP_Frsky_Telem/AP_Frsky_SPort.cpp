#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_Frsky_SPort.h"

#include <string.h>

extern const AP_HAL::HAL& hal;

bool AP_Frsky_SPort::should_process_packet(const uint8_t *packet, bool discard_duplicates)
{
    // check for duplicate packets
    if (discard_duplicates && _parse_state.last_packet != nullptr) {
        /*
          Note: the polling byte packet[0] should be ignored in the comparison
          because we might get the same packet with different polling bytes
          We have 2 types of duplicate packets: ghost identical packets sent by the receiver
          and user duplicate packets sent to compensate for bad link and frame loss, this
          check should address both types.
        */
        if (memcmp(&packet[1], &_parse_state.last_packet[1], SPORT_PACKET_SIZE-1) == 0) {
            return false;
        }
        memcpy(_parse_state.last_packet, packet, SPORT_PACKET_SIZE);
    }
    //check CRC
    int16_t crc = 0;
    for (uint8_t i=1; i<SPORT_PACKET_SIZE; ++i) {
        crc += _parse_state.rx_buffer[i]; // 0-1FE
        crc += crc >> 8;  // 0-1FF
        crc &= 0x00ff;    // 0-FF
    }
    return (crc == 0x00ff);
}

bool AP_Frsky_SPort::process_byte(sport_packet_t &sport_packet, const uint8_t data)
{
    switch (_parse_state.state) {
    case ParseState::START:
        if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data;
        }
        _parse_state.state = ParseState::IN_FRAME;
        break;

    case ParseState::IN_FRAME:
        if (data == FRAME_DLE) {
            _parse_state.state = ParseState::XOR; // XOR next byte
        } else if (data == FRAME_HEAD) {
            _parse_state.state = ParseState::IN_FRAME ;
            _parse_state.rx_buffer_count = 0;
            break;
        } else if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data;
        }
        break;

    case ParseState::XOR:
        if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data ^ STUFF_MASK;
        }
        _parse_state.state = ParseState::IN_FRAME;
        break;

    case ParseState::IDLE:
        if (data == FRAME_HEAD) {
            _parse_state.rx_buffer_count = 0;
            _parse_state.state = ParseState::START;
        }
        break;

    } // switch

    if (_parse_state.rx_buffer_count >= SPORT_PACKET_SIZE) {
        _parse_state.rx_buffer_count = 0;
        _parse_state.state = ParseState::IDLE;
        // feed the packet only if it's not a duplicate
        return get_packet(sport_packet, true);
    }
    return false;
}

bool AP_Frsky_SPort::get_packet(sport_packet_t &sport_packet, bool discard_duplicates)
{
    if (!should_process_packet(_parse_state.rx_buffer, discard_duplicates)) {
        return false;
    }

    const sport_packet_t sp {
        _parse_state.rx_buffer[0],
        _parse_state.rx_buffer[1],
        le16toh_ptr(&_parse_state.rx_buffer[2]),
        le32toh_ptr(&_parse_state.rx_buffer[4])
    };

    sport_packet = sp;
    return true;
}

/*
 * Calculates the sensor id from the physical sensor index [0-27]
        0x00, 	// Physical ID 0 - Vario2 (altimeter high precision)
        0xA1, 	// Physical ID 1 - FLVSS Lipo sensor
        0x22, 	// Physical ID 2 - FAS-40S current sensor
        0x83, 	// Physical ID 3 - GPS / altimeter (normal precision)
        0xE4, 	// Physical ID 4 - RPM
        0x45, 	// Physical ID 5 - SP2UART(Host)
        0xC6, 	// Physical ID 6 - SPUART(Remote)
        0x67, 	// Physical ID 7 - Ardupilot/Betaflight EXTRA DOWNLINK
        0x48, 	// Physical ID 8 -
        0xE9, 	// Physical ID 9 -
        0x6A, 	// Physical ID 10 -
        0xCB, 	// Physical ID 11 -
        0xAC, 	// Physical ID 12 -
        0x0D, 	// Physical ID 13 - Ardupilot/Betaflight UPLINK
        0x8E, 	// Physical ID 14 -
        0x2F, 	// Physical ID 15 -
        0xD0, 	// Physical ID 16 -
        0x71, 	// Physical ID 17 -
        0xF2, 	// Physical ID 18 -
        0x53, 	// Physical ID 19 -
        0x34, 	// Physical ID 20 - Ardupilot/Betaflight EXTRA DOWNLINK
        0x95, 	// Physical ID 21 -
        0x16, 	// Physical ID 22 - GAS Suite
        0xB7, 	// Physical ID 23 - IMU ACC (x,y,z)
        0x98, 	// Physical ID 24 -
        0x39, 	// Physical ID 25 - Power Box
        0xBA 	// Physical ID 26 - Temp
        0x1B	// Physical ID 27 - ArduPilot/Betaflight DEFAULT DOWNLINK
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
#define BIT(x, index) (((x) >> index) & 0x01)
uint8_t AP_Frsky_SPort::calc_sport_sensor_id(const uint8_t physical_id)
{
    uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}

