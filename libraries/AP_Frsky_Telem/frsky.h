#pragma once

#include <AP_HAL/UARTDriver.h>
#include <stdint.h>

/*
for FrSky D protocol (D-receivers)
*/
// FrSky sensor hub data IDs
#define DATA_ID_GPS_ALT_BP          0x01
#define DATA_ID_TEMP1               0x02
#define DATA_ID_FUEL                0x04
#define DATA_ID_TEMP2               0x05
#define DATA_ID_GPS_ALT_AP          0x09
#define DATA_ID_BARO_ALT_BP         0x10
#define DATA_ID_GPS_SPEED_BP        0x11
#define DATA_ID_GPS_LONG_BP         0x12
#define DATA_ID_GPS_LAT_BP          0x13
#define DATA_ID_GPS_COURS_BP        0x14
#define DATA_ID_GPS_SPEED_AP        0x19
#define DATA_ID_GPS_LONG_AP         0x1A
#define DATA_ID_GPS_LAT_AP          0x1B
#define DATA_ID_BARO_ALT_AP         0x21
#define DATA_ID_GPS_LONG_EW         0x22
#define DATA_ID_GPS_LAT_NS          0x23
#define DATA_ID_CURRENT             0x28
#define DATA_ID_VARIO               0x30
#define DATA_ID_VFAS                0x39

#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D

/*
for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
// FrSky Sensor IDs
#define SENSOR_ID_VARIO             0x00 // Sensor ID  0
#define SENSOR_ID_FAS               0x22 // Sensor ID  2
#define SENSOR_ID_GPS               0x83 // Sensor ID  3
#define SENSOR_ID_SP2UR             0xC6 // Sensor ID  6
#define SENSOR_ID_27                0x1B // Sensor ID 27
// two way protocol sensor ids
#define SENSOR_ID_UPLINK            0x0D
#define SENSOR_ID_1_DOWNLINK        0x34
#define SENSOR_ID_2_DOWNLINK        0x67

// FrSky data IDs
#define GPS_LONG_LATI_FIRST_ID      0x0800
#define DIY_FIRST_ID                0x5000

#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20

#define SPORT_DATA_FRAME            0x10

/*
for FrSky SPort Passthrough
*/
// data bits preparation
// for parameter data
#define PARAM_ID_OFFSET             24
#define PARAM_VALUE_LIMIT           0xFFFFFF
// for gps status data
#define GPS_SATS_LIMIT              0xF
#define GPS_STATUS_LIMIT            0x3
#define GPS_STATUS_OFFSET           4
#define GPS_HDOP_OFFSET             6
#define GPS_ADVSTATUS_OFFSET        14
#define GPS_ALTMSL_OFFSET           22
// for battery data
#define BATT_VOLTAGE_LIMIT          0x1FF
#define BATT_CURRENT_OFFSET         9
#define BATT_TOTALMAH_LIMIT         0x7FFF
#define BATT_TOTALMAH_OFFSET        17
// for autopilot status data
#define AP_CONTROL_MODE_LIMIT       0x1F
#define AP_SIMPLE_OFFSET            5
#define AP_SSIMPLE_OFFSET           6
#define AP_FLYING_OFFSET            7
#define AP_ARMED_OFFSET             8
#define AP_BATT_FS_OFFSET           9
#define AP_EKF_FS_OFFSET            10
#define AP_IMU_TEMP_MIN             19.0f
#define AP_IMU_TEMP_MAX             82.0f
#define AP_IMU_TEMP_OFFSET          26
// for home position related data
#define HOME_ALT_OFFSET             12
#define HOME_BEARING_LIMIT          0x7F
#define HOME_BEARING_OFFSET         25
// for velocity and yaw data
#define VELANDYAW_XYVEL_OFFSET      9
#define VELANDYAW_YAW_LIMIT         0x7FF
#define VELANDYAW_YAW_OFFSET        17
// for attitude (roll, pitch) and range data
#define ATTIANDRNG_ROLL_LIMIT       0x7FF
#define ATTIANDRNG_PITCH_LIMIT      0x3FF
#define ATTIANDRNG_PITCH_OFFSET     11
#define ATTIANDRNG_RNGFND_OFFSET    21
// for two way communication
#define SPORT_MAVLITE_MSG_SIZE(LEN) ((uint8_t)(1 + ceilf((LEN-2)/5.0f))) // number of sport packets required to transport a message with LEN payload
#define TELEMETRY_RX_BUFFER_SIZE    19U  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
#define SPORT_PACKET_BUFFER_SIZE    ((uint8_t)30U*SPORT_MAVLITE_MSG_SIZE(MAVLITE_MAX_PAYLOAD_LEN))
#define SPORT_PACKET_SIZE           9U
#define STUFF_MASK                  0x20
#define SPORT_DATA_FRAME            0x10
#define SPORT_UPLINK_FRAME          0x30
#define SPORT_UPLINK_FRAME_RW       0x31
#define SPORT_DOWNLINK_FRAME        0x32

#define FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY          5 // size of the message buffer queue (max number of messages waiting to be sent)

namespace FRSky
{
typedef union {
    struct PACKED {
        uint8_t sensor;
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
    };
    uint8_t raw[8];
} sport_packet_t;

typedef enum : uint8_t {
    STATE_DATA_IDLE,
    STATE_DATA_START,
    STATE_DATA_IN_FRAME,
    STATE_DATA_XOR,
} sport_parse_state_e;

typedef struct PACKED {
    uint8_t telemetry_rx_buffer_count;
    uint8_t telemetry_rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
    sport_parse_state_e state;
} sport_parse_state_t;

// serial tx helpers
void send_byte(AP_HAL::UARTDriver *port, uint8_t value);
void send_uint16(AP_HAL::UARTDriver *port, uint16_t id, uint16_t data);
void send_sport_frame(AP_HAL::UARTDriver *port, uint8_t frame, uint16_t appid, uint32_t data);
// packet parser helpers
bool parse_sport_telemetry_data(uint8_t data, sport_parse_state_t &parse_state);
bool check_sport_packet(const uint8_t *packet);
bool check_sport_packet(const uint8_t *packet, const uint8_t *last_packet);
// sensor id helper
uint8_t get_sport_sensor_id(uint8_t physical_id);
}