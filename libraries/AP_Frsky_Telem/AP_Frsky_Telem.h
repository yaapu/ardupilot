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
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RCTelemetry/AP_RCTelemetry.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Frsky_Parameters.h"
#include "frsky.h"

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 1
#endif


#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include "mavlite.h"
using namespace MAVLITE;
// for fair scheduler
#define TIME_SLOT_MAX               12U
#else
// for fair scheduler
#define TIME_SLOT_MAX               11U
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
using namespace FRSky;

class AP_Frsky_Telem : public AP_RCTelemetry
{
public:
    AP_Frsky_Telem(bool external_data=false);

    ~AP_Frsky_Telem();

    /* Do not allow copies */
    AP_Frsky_Telem(const AP_Frsky_Telem &other) = delete;
    AP_Frsky_Telem &operator=(const AP_Frsky_Telem&) = delete;

    // init - perform required initialisation
    virtual bool init() override;

    static AP_Frsky_Telem *get_singleton(void)
    {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data
    static bool set_telem_data(const uint8_t frame,const uint16_t appid, const uint32_t data);
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
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
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        MAV =           11  // mavlite
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    };

    struct {
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

    struct PACKED {
        bool send_latitude; // sizeof(bool) = 4 ?
        uint32_t gps_lng_sample;
        uint8_t new_byte;
    } _passthrough;


#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    struct {
        uint8_t uplink_sensor_id = 0x0D;
        uint8_t downlink1_sensor_id = 0x34;
        uint8_t downlink2_sensor_id = 0x67;
    } _sport_config;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

    struct {
        bool sport_status;
        bool gps_refresh;
        bool vario_refresh;
        uint8_t fas_call;
        uint8_t gps_call;
        uint8_t vario_call;
        uint8_t various_call;
    } _SPort;

    struct {
        uint32_t last_200ms_frame;
        uint32_t last_1000ms_frame;
    } _D;

    struct {
        uint32_t chunk; // a "chunk" (four characters/bytes) at a time of the queued message to be sent
        uint8_t repeats; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
        uint8_t char_index; // index of which character to get in the message
    } _msg_chunk;

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    AP_Frsky_Parameters* _frsky_parameters;
    // bidirectional sport telemetry
    ObjectBuffer_TS<sport_packet_t> _sport_rx_packet_buffer{SPORT_PACKET_BUFFER_SIZE};
    ObjectBuffer_TS<sport_packet_t> _sport_tx_packet_buffer{SPORT_PACKET_BUFFER_SIZE};

    uint8_t _last_sport_packet[SPORT_PACKET_SIZE];
    sport_parse_state_t _parse_state;

    mavlite_message_t _mavlite_rx_message;
    mavlite_message_t _last_mavlite_rx_message; // prevent duplicate incoming messages?
    mavlite_status_t _mavlite_rx_status;

    mavlite_message_t _mavlite_tx_message;
    mavlite_status_t _mavlite_tx_status;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // passthrough WFQ scheduler
    void setup_wfq_scheduler(void) override;
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;

    // main transmission function when protocol is FrSky SPort Passthrough (OpenTX)
    void send_SPort_Passthrough(void);
    // main transmission function when protocol is FrSky SPort
    void send_SPort(void);
    // main transmission function when protocol is FrSky D
    void send_D(void);
    // main call to send updates to transmitter (called by scheduler at 1kHz)
    void loop(void);

    // methods related to the nuts-and-bolts of sending data
    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    float get_vspeed_ms(void);
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
    float format_gps(float dec);
    void calc_nav_alt(void);
    void calc_gps_position(void);

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    bool set_sport_sensor_id(AP_Int8 idx, uint8_t &sensor);
    // tx/rx sport packet processing
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
    MAV_RESULT mavlite_handle_command_preflight_calibration_baro();
    MAV_RESULT mavlite_handle_command_do_fence_enable(uint16_t param1);
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

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

    // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data (internal function)
    bool _set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data);
#endif
};

namespace AP
{
AP_Frsky_Telem *frsky_telem();
};
