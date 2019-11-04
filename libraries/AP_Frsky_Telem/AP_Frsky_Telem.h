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

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_RCTelemetry/AP_RCTelemetry.h>

#include "AP_Frsky_Parameters.h"
#include "mavlite.h"

#define FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY          5 // size of the message buffer queue (max number of messages waiting to be sent)

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
//#define START_STOP_SPORT            0x7E
//#define BYTESTUFF_SPORT             0x7D

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
// for fair scheduler
#define TIME_SLOT_MAX               12
// for two way communication
#define SPORT_MAVLITE_MSG_SIZE(LEN) ((uint8_t)(1 + ceilf((LEN-2)/5.0f))) // number of sport packets required to transport a message with LEN payload
#define TELEMETRY_RX_BUFFER_SIZE    19  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
#define SPORT_PACKET_BUFFER_SIZE    30*SPORT_MAVLITE_MSG_SIZE(MAVLITE_MAX_PAYLOAD_LEN)
#define SPORT_PACKET_SIZE           9
#define STUFF_MASK                  0x20
#define SPORT_DATA_FRAME            0x10
#define SPORT_UPLINK_FRAME          0x30
#define SPORT_UPLINK_FRAME_RW       0x31
#define SPORT_DOWNLINK_FRAME        0x32

#define ENABLE_SCHED_STATS
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define ENABLE_SCHED_STATS
#endif

using namespace MAVLITE;

// forward declaration to solve circular reference, GCS.h includes AP_Frsky_Telem.h!
class GCS_MAVLINK_Frsky;

class AP_Frsky_Telem : public AP_RCTelemetry {
public:
    AP_Frsky_Telem(bool external_data=false);

    ~AP_Frsky_Telem();

    typedef union{
        struct PACKED
        {
            uint8_t sensor;
            uint8_t frame;
            uint16_t appid;
            uint32_t data;
        };
        uint8_t raw[8];
    } sport_packet_t;

    /* Do not allow copies */
    AP_Frsky_Telem(const AP_Frsky_Telem &other) = delete;
    AP_Frsky_Telem &operator=(const AP_Frsky_Telem&) = delete;

    // init - perform required initialisation
    virtual bool init() override;

    static AP_Frsky_Telem *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);
    // set telemetry data from external producer of SPort data
    static bool set_telem_data(const uint8_t frame,const uint16_t appid, const uint32_t data);
    
private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    uint16_t _crc;

    uint8_t _sport_tx_packet_duplicates = 0;
    uint8_t _paramID;
    
    enum PassthroughPacketType : uint8_t {
        TEXT =          0,  // 0x5000 status text (dynamic)
        ATTITUDE =      1,  // 0x5006 Attitude and range (dynamic)
        GPS_LAT =       2,  // 0x800 GPS lat
        GPS_LON =       3,  // 0x800 GPS lon
        VEL_YAW =       4,  // 0x5005 Vel and Yaw
        AP_STATUS =     5,  // 0x5001 AP status
        GPS_STATUS =    6,  // 0x5002 GPS status
        HOME =          7,  // 0x5004 Home
        BATT_2 =        8,  // 0x5008 Battery 2 status
        BATT_1 =        9,  // 0x5008 Battery 1 status
        PARAM =         10, // 0x5007 parameters
        MAV =           11  // 0x5007 parameters
    };

    typedef struct {
        HAL_Semaphore sem;
        ObjectBuffer<sport_packet_t> queue{SPORT_PACKET_BUFFER_SIZE};
    } sport_packet_buffer_t;

    struct
    {
        int32_t vario_vspd;
        char lat_ns, lon_ew;
        uint16_t latdddmm;
        uint16_t latmmmm;
        uint16_t londddmm;
        uint16_t lonmmmm;
        uint16_t alt_gps_meters;
        uint16_t alt_gps_cm;
        uint16_t alt_nav_meters;
        uint16_t alt_nav_cm;
        int16_t speed_in_meter;
        uint16_t speed_in_centimeter;
        uint16_t yaw;
    } _SPort_data;

    // Receive buffer state machine state enum
    typedef enum : uint8_t {
        STATE_DATA_IDLE,
        STATE_DATA_START,
        STATE_DATA_IN_FRAME,
        STATE_DATA_XOR,
    } sport_parse_state_t;

    struct PACKED
    {
        bool send_latitude; // sizeof(bool) = 4 ?
        uint32_t gps_lng_sample;
        uint8_t new_byte;
        sport_parse_state_t sport_parse_state = STATE_DATA_IDLE; // sizeof(sport_parse_state_t) = 1 
    } _passthrough;
    

    struct
    {
        uint8_t uplink_sensor_id = 0x0D;
        uint8_t downlink1_sensor_id = 0x34;
        uint8_t downlink2_sensor_id = 0x67;
    } _sport_config;

    struct
    {
        bool sport_status;
        bool gps_refresh;
        bool vario_refresh;
        uint8_t fas_call;
        uint8_t gps_call;
        uint8_t vario_call;
        uint8_t various_call;
    } _SPort;
    
    struct
    {
        uint32_t last_200ms_frame;
        uint32_t last_1000ms_frame;
    } _D;
    
    struct
    {
        uint32_t chunk; // a "chunk" (four characters/bytes) at a time of the queued message to be sent
        uint8_t repeats; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
        uint8_t char_index; // index of which character to get in the message
    } _msg_chunk;

    GCS_MAVLINK_Frsky* _gcs_mavlink_frsky;
    AP_Frsky_Parameters* _frsky_parameters;
    
    float get_vspeed_ms(void);
    
    // bidirectional sport telemetry
    sport_packet_buffer_t _sport_rx_packet_buffer;
    sport_packet_buffer_t _sport_tx_packet_buffer;

    uint8_t _telemetry_rx_buffer_count;
    uint8_t _telemetry_rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
    uint8_t _last_sport_packet[SPORT_PACKET_SIZE];

    mavlite_message_t _mavlite_rx_message;
    mavlite_message_t _last_mavlite_rx_message; // prevent duplicate incoming messages?
    mavlite_status_t _mavlite_rx_status;
    
    mavlite_message_t _mavlite_tx_message;
    mavlite_status_t _mavlite_tx_status;

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;
    // main transmission function when protocol is FrSky SPort Passthrough (OpenTX)
    void send_SPort_Passthrough(void);
    // main transmission function when protocol is FrSky SPort
    void send_SPort(void);
    // main transmission function when protocol is FrSky D
    void send_D(void);
    // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
    void loop(void);
    // methods related to the nuts-and-bolts of sending data
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);
    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    bool get_next_msg_chunk(void) override;
    uint32_t calc_param(void);
    uint32_t calc_gps_latlng(bool *send_latitude);
    uint32_t calc_gps_status(void);
    uint32_t calc_batt(uint8_t instance);
    uint32_t calc_ap_status(void);
    uint32_t calc_home(void);
    uint32_t calc_velandyaw(void);
    uint32_t calc_attiandrng(void);
    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);

    // methods to convert flight controller data to FrSky D or SPort format
    void calc_nav_alt(void);
    float format_gps(float dec);
    void calc_gps_position(void);

    // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);
    // set telemetry data from external producer of SPort data (internal function)
    bool _set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data);
    
    static AP_Frsky_Telem *singleton;

    // use_external_data is set when this library will
    // be providing data to another transport, such as FPort
    bool use_external_data;
    struct {
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
        bool pending;
    } external_data;

    // sport data tx/rx methods
    bool check_sport_packet(const uint8_t *packet);
    uint8_t get_sport_sensor_id(uint8_t physical_id);
    bool set_sport_sensor_id(AP_Int8 idx, uint8_t &sensor);
    void process_sport_telemetry_data(uint8_t data);
    void process_sport_telemetry_packet(const uint8_t *packet);
    void process_sport_rx_queue();
    void process_sport_tx_queue();

    // mavlite messages tx/rx methods
    bool mavlite_send_message(mavlite_message_t* rxmsg, mavlite_status_t* status);
    void mavlite_process_message(mavlite_message_t* rxmsg);
    
    // gcs mavlite methods
    void mavlite_handle_param_request_read(mavlite_message_t* rxmsg);
    void mavlite_handle_param_set(mavlite_message_t* rxmsg);
    void mavlite_handle_command_long(mavlite_message_t* rxmsg);
};

namespace AP {
    AP_Frsky_Telem *frsky_telem();
};
