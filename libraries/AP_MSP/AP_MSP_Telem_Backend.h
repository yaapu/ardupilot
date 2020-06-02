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

#include <AP_RCTelemetry/AP_RCTelemetry.h>
#include <time.h>
#include "msp.h"

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_MSP_ENABLED

#define MSP_TIME_SLOT_MAX 12
#define CELLFULL 4.35
#define MSP_TXT_BUFFER_SIZE 14U // 11 + 3 utf8 chars
#define MSP_TXT_VISIBLE_CHARS 12U
#define MSP_ARROW_COUNT 8

using namespace MSP;

class AP_MSP;

class AP_MSP_Telem_Backend : AP_RCTelemetry
{
public:
    AP_MSP_Telem_Backend(AP_MSP &msp, uint8_t protocol_instance, bool scheduler_enabled);

    ~AP_MSP_Telem_Backend();

    typedef struct telemetry_info_s {
        int32_t roll_cd;
        int32_t pitch_cd;
        float yaw_deg;

        bool home_is_set;
        float home_bearing_cd;
        uint32_t home_distance_m;

        float vspeed_ms;
        int32_t rel_altitude_cm;

        float batt_current_a;
        float batt_consumed_mah;
        float batt_voltage_v;
        int32_t batt_capacity_mah;
        uint8_t batt_cellcount;
        battery_state_e batt_state;

        int32_t gps_latitude;
        int32_t gps_longitude;
        uint8_t gps_num_sats;
        int32_t gps_altitude_cm;
        float gps_speed_ms;
        uint16_t gps_ground_course_cd;
        uint8_t gps_fix_type;
        float airspeed_estimate_ms;
        bool  airspeed_have_estimate;

        char flight_mode_str[MSP_TXT_BUFFER_SIZE+1];    // allow space for the terminator

        tm localtime_tm;                                // year is relative to 1900
    } telemetry_info_t;

    // init - perform required initialisation
    virtual bool init() override;
    // scheduler
    void run_wfq_scheduler();

    // telemetry helpers
    static uint8_t calc_cell_count(float battery_voltage);
    static float get_vspeed_ms(void);
    static void telem_update_home_pos(telemetry_info_t &_telemetry);
    static void telem_update_battery_state(telemetry_info_t &_telemetry);
    static void telem_update_gps_state(telemetry_info_t &_telemetry);
    static void telem_update_attitude(telemetry_info_t &_telemetry);
    static void telem_update_airspeed(telemetry_info_t &_telemetry);
    static void telem_update_flight_mode(telemetry_info_t &_telemetry);
    static void telem_update_localtime(telemetry_info_t &_telemetry);
    static void telem_update_wind(telemetry_info_t &_telemetry);

    // implementation specific helpers
    virtual uint32_t get_osd_flight_mode_bitmask(void) = 0;
    virtual void flash_osd_items(void);
    virtual void apply_osd_items_overrides(void);
    virtual void enable_warnings();

    // implementation specific MSP out command processing
    virtual bool msp_process_out_api_version(sbuf_t *dst) = 0;
    virtual bool msp_process_out_fc_version(sbuf_t *dst) = 0;
    virtual bool msp_process_out_fc_variant(sbuf_t *dst) = 0;
    virtual bool msp_process_out_uid(sbuf_t *dst);

    virtual bool msp_process_out_board_info(sbuf_t *dst);
    virtual bool msp_process_out_build_info(sbuf_t *dst);
    virtual bool msp_process_out_name(sbuf_t *dst);
    virtual bool msp_process_out_status(sbuf_t *dst);
    virtual bool msp_process_out_osd_config(sbuf_t *dst);
    virtual bool msp_process_out_raw_gps(sbuf_t *dst);
    virtual bool msp_process_out_comp_gps(sbuf_t *dst);
    virtual bool msp_process_out_attitude(sbuf_t *dst);
    virtual bool msp_process_out_altitude(sbuf_t *dst);
    virtual bool msp_process_out_analog(sbuf_t *dst);
    virtual bool msp_process_out_battery_state(sbuf_t *dst);
    virtual bool msp_process_out_esc_sensor_data(sbuf_t *dst);
    virtual bool msp_process_out_rtc(sbuf_t *dst);
    virtual bool msp_process_out_rc(sbuf_t *dst);

protected:
    enum msp_packet_type : uint8_t {
        EMPTY_SLOT = 0,
        NAME,
        STATUS,
        CONFIG,
        RAW_GPS,
        COMP_GPS,
        ATTITUDE,
        ALTITUDE,
        ANALOG,
        BATTERY_STATE,
        ESC_SENSOR_DATA,
        RTC_DATETIME,
    };

    const uint16_t msp_packet_type_map[MSP_TIME_SLOT_MAX] = {
        0,
        MSP_NAME,
        MSP_STATUS,
        MSP_OSD_CONFIG,
        MSP_RAW_GPS,
        MSP_COMP_GPS,
        MSP_ATTITUDE,
        MSP_ALTITUDE,
        MSP_ANALOG,
        MSP_BATTERY_STATE,
        MSP_ESC_SENSOR_DATA,
        MSP_RTC
    };
    /* UTF-8 encodings
        U+2191 ↑       e2 86 91        UPWARDS ARROW
        U+2197 ↗       e2 86 97        NORTH EAST ARROW
        U+2192 →       e2 86 92        RIGHTWARDS ARROW
        U+2198 ↘       e2 86 98        SOUTH EAST ARROW
        U+2193 ↓       e2 86 93        DOWNWARDS ARROW
        U+2199 ↙       e2 86 99        SOUTH WEST ARROW
        U+2190 ←       e2 86 90        LEFTWARDS ARROW
        U+2196 ↖       e2 86 96        NORTH WEST ARROW
    */
    static constexpr uint8_t arrows[8] = {0x91, 0x97, 0x92, 0x98, 0x93, 0x99, 0x90, 0x96};

    //static const uint8_t message_visible_width = 12;
    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    // reference to creator for callback
    AP_MSP& _msp;
    // should we have push type telemetry
    bool _scheduler_enabled;
    // serial port instance
    uint8_t _protocol_instance;
    // each backend can hide/unhide items dynamically
    uint64_t osd_hidden_items_bitmask;


    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    void setup_wfq_scheduler(void) override;
    bool get_next_msg_chunk(void) override;
};

#endif  //HAL_MSP_ENABLED