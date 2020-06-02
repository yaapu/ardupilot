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

#include <AP_OSD/AP_OSD.h>

#include "AP_MSP_Telem_Backend.h"

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_MSP_ENABLED

#define MSP_MAX_INSTANCES 3
#define ARMING_DISABLED_ARM_SWITCH 1 << 24
#define ARMING_DISABLE_FLAGS_COUNT 25

#define MSP_OSD_START 2048
#define MSP_OSD_STEP_X 1
#define MSP_OSD_STEP_Y 32
#define MSP_OSD_POS(osd_setting) (MSP_OSD_START + osd_setting->xpos*MSP_OSD_STEP_X + osd_setting->ypos*MSP_OSD_STEP_Y)

using namespace MSP;

class AP_MSP
{
public:
    AP_MSP();

    ~AP_MSP();

    /* Do not allow copies */
    AP_MSP(const AP_MSP &other) = delete;
    AP_MSP &operator=(const AP_MSP&) = delete;

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init - perform required initialisation
    bool init();
    void msp_telemetry_callback(uint16_t cmd_msp, uint8_t protocol_instance);

    static AP_MSP *get_singleton(void)
    {
        return singleton;
    }

private:
    // to access the telemetry_info member variable
    friend class AP_MSP_Telem;
    friend class AP_MSP_Telem_DJI;
    friend class AP_MSP_Telem_Backend;

    AP_OSD_Setting rssi     {true, 20, 0};       // OSD_RSSI
    AP_OSD_Setting vbat     {true, 1, 5};        // OSD_MAIN_BATT_VOLTAGE
    AP_OSD_Setting ahor     {true, 14, 3};       // OSD_ARTIFICIAL_HORIZON
    AP_OSD_Setting ahbars   {true, 14, 7};       // OSD_HORIZON_SIDEBARS
    AP_OSD_Setting cross    {true, 14, 7};       // OSD_HORIZON_SIDEBARS
    AP_OSD_Setting flmode   {false, 1, 1};       // OSD_FLYMODE - not used for now
    AP_OSD_Setting name     {true, 10, 9};       // OSD_CRAFT_NAME
    AP_OSD_Setting curr     {true, 0, 12};       // OSD_CURRENT_DRAW
    AP_OSD_Setting mah      {true, 0, 11};       // OSD_MAH_DRAWN
    AP_OSD_Setting gspd     {true, 21, 14};      // OSD_GPS_SPEED
    AP_OSD_Setting nsats    {true, 1, 0};        // OSD_GPS_SATS
    AP_OSD_Setting alt      {true, 21, 13};      // OSD_ALTITUDE
    AP_OSD_Setting cell     {true, 1, 7};        // OSD_AVG_CELL_VOLTAGE
    AP_OSD_Setting lon      {true, 9, 0};        // OSD_GPS_LON
    AP_OSD_Setting lat      {true, 9, 1};        // OSD_GPS_LAT
    AP_OSD_Setting pitch    {true, 23, 6};       // OSD_PITCH_ANGLE
    AP_OSD_Setting roll     {true, 23, 5};       // OSD_ROLL_ANGLE
    AP_OSD_Setting yaw      {true, 23, 7};       // OSD_NUMERICAL_HEADING
    AP_OSD_Setting battusg  {true, 9, 15};       // OSD_MAIN_BATT_USAGE
    AP_OSD_Setting hdir     {true, 0, 10};       // OSD_HOME_DIR
    AP_OSD_Setting hdist    {true, 2, 10};       // OSD_HOME_DIST
    AP_OSD_Setting vario    {true, 22, 12};      // OSD_NUMERICAL_VARIO
    AP_OSD_Setting arm      {true, 10, 10};      // OSD_DISARMED
    AP_OSD_Setting power    {true, 0, 13};       // OSD_POWER
    AP_OSD_Setting rtc      {true, 9, 2};        // OSD_RTC_DATETIME
    AP_OSD_Setting esctemp  {true, 0, 9};        // OSD_ESC_TMP
    AP_OSD_Setting warn     {true, 10, 11};      // OSD_WARNINGS
    AP_OSD_Setting throttle {true, 23, 8};       // OSD_THROTTLE_POS

    enum : uint8_t {
        BACKEND_MSP,
        BACKEND_DJI,
        BACKEND_COUNT
    };

    AP_Int8 units;
    AP_Int8 msgtime_s;
    AP_Int8 cellcount;
    AP_Int8 wind_en;
    AP_Int8 airspeed_en;

    // these are the osd items we support for MSP OSD
    AP_OSD_Setting* osd_item_settings[58] = {
        &rssi,              // OSD_RSSI_VALUE
        &vbat,              // OSD_MAIN_BATT_VOLTAGE
        &cross,             // OSD_CROSSHAIRS
        &ahor,              // OSD_ARTIFICIAL_HORIZON
        &ahbars,            // OSD_HORIZON_SIDEBARS
        nullptr,            // OSD_ITEM_TIMER_1
        nullptr,            // OSD_ITEM_TIMER_2
        nullptr,            // OSD_FLYMODE disabled for now
        &name,              // OSD_CRAFT_NAME
        &throttle,          // OSD_THROTTLE_POS
        nullptr,            // OSD_VTX_CHANNEL
        &curr,              // OSD_CURRENT_DRAW
        &mah,               // OSD_MAH_DRAWN
        &gspd,              // OSD_GPS_SPEED
        &nsats,             // OSD_GPS_SATS
        &alt,               // OSD_ALTITUDE
        nullptr,            // OSD_ROLL_PIDS
        nullptr,            // OSD_PITCH_PIDS
        nullptr,            // OSD_YAW_PIDS
        &power,             // OSD_POWER
        nullptr,            // OSD_PIDRATE_PROFILE
        &warn,              // OSD_WARNINGS
        &cell,              // OSD_AVG_CELL_VOLTAGE
        &lon,               // OSD_GPS_LON
        &lat,               // OSD_GPS_LAT
        nullptr,            // OSD_DEBUG
        &pitch,             // OSD_PITCH_ANGLE
        &roll,              // OSD_ROLL_ANGLE
        &battusg,           // OSD_MAIN_BATT_USAGE
        &arm,               // OSD_DISARMED
        &hdir,              // OSD_HOME_DIR
        &hdist,             // OSD_HOME_DIST
        &yaw,               // OSD_NUMERICAL_HEADING
        &vario,             // OSD_NUMERICAL_VARIO
        nullptr,            // OSD_COMPASS_BAR
#ifdef HAVE_AP_BLHELI_SUPPORT
        &esctemp,           // OSD_ESC_TMP
#else
        nullptr,            // OSD_ESC_TMP
#endif
        nullptr,            // OSD_ESC_RPM
        nullptr,            // OSD_REMAINING_TIME_ESTIMATE
        &rtc,               // OSD_RTC_DATETIME
        nullptr,            // OSD_ADJUSTMENT_RANGE
        nullptr,            // OSD_CORE_TEMPERATURE
        nullptr,            // OSD_ANTI_GRAVITY
        nullptr,            // OSD_G_FORCE
        nullptr,            // OSD_MOTOR_DIAG
        nullptr,            // OSD_LOG_STATUS
        nullptr,            // OSD_FLIP_ARROW
        nullptr,            // OSD_LINK_QUALITY
        nullptr,            // OSD_FLIGHT_DIST
        nullptr,            // OSD_STICK_OVERLAY_LEFT
        nullptr,            // OSD_STICK_OVERLAY_RIGHT
        nullptr,            // OSD_DISPLAY_NAME
        nullptr,            // OSD_ESC_RPM_FREQ
        nullptr,            // OSD_RATE_PROFILE_NAME
        nullptr,            // OSD_PID_PROFILE_NAME
        nullptr,            // OSD_PROFILE_NAME
        nullptr,            // OSD_RSSI_DBM_VALUE
        nullptr,            // OSD_RC_CHANNELS
        nullptr,            // OSD_CAMERA_FRAME,
    };

    const int osd_enabled_stats[24] = {
        -1,                         // OSD_STAT_RTC_DATE_TIME
            -1,                         // OSD_STAT_TIMER_1
            -1,                         // OSD_STAT_TIMER_2
            -1,                         // OSD_STAT_MAX_SPEED
            -1,                         // OSD_STAT_MAX_DISTANCE
            -1,                         // OSD_STAT_MIN_BATTERY
            -1,                         // OSD_STAT_END_BATTERY
            -1,                         // OSD_STAT_BATTERY
            -1,                         // OSD_STAT_MIN_RSSI
            -1,                         // OSD_STAT_MAX_CURRENT
            -1,                         // OSD_STAT_USED_MAH
            -1,                         // OSD_STAT_MAX_ALTITUDE
            -1,                         // OSD_STAT_BLACKBOX
            -1,                         // OSD_STAT_BLACKBOX_NUMBER
            -1,                         // OSD_STAT_MAX_G_FORCE
            -1,                         // OSD_STAT_MAX_ESC_TEMP
            -1,                         // OSD_STAT_MAX_ESC_RPM
            -1,                         // OSD_STAT_MIN_LINK_QUALITY
            -1,                         // OSD_STAT_FLIGHT_DISTANCE
            -1,                         // OSD_STAT_MAX_FFT
            -1,                         // OSD_STAT_TOTAL_FLIGHTS
            -1,                         // OSD_STAT_TOTAL_TIME
            -1,                         // OSD_STAT_TOTAL_DIST
            -1,                         // OSD_STAT_MIN_RSSI_DBM
        };

    AP_MSP_Telem_Backend::telemetry_info_t telemetry_info;
    osd_config_t osd_config;
    msp_port_t msp_port[MSP_MAX_INSTANCES];
    // OSD item flashing support
    uint32_t flash_start_time;
    bool flashing_on;
    // we need to detect flight mode changes
    uint8_t last_flight_mode = 255;
    uint32_t last_flight_mode_change = AP_HAL::millis();
    bool flight_mode_focus;


    void loop(void);
    void process_serial_data(uint8_t instance);

    // MSP protocol decoder
    void msp_process_received_command(msp_port_t *msp);
    msp_result_e msp_process_command(msp_port_t* msp, msp_packet_t *cmd, msp_packet_t *reply);
    msp_result_e msp_process_sensor_command(msp_port_t* msp, uint16_t cmd_msp, sbuf_t *src);
    bool msp_process_out_command(msp_port_t* msp, uint16_t cmd_msp, sbuf_t *dst);

    // MSP sensor command processing
    void msp_handle_opflow(const msp_opflow_sensor_t &pkt);
    void msp_handle_rangefinder(const msp_rangefinder_sensor_t &pkt);

    static AP_MSP *singleton;
};

namespace AP
{
AP_MSP *msp();
};

#endif  //HAL_MSP_ENABLED