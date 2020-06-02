#pragma once

#include <stdint.h>

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_MSP_ENABLED

#define OSD_FLAGS_OSD_FEATURE           (1 << 0)

namespace MSP {
    typedef enum {
        OSD_RSSI_VALUE,
        OSD_MAIN_BATT_VOLTAGE,
        OSD_CROSSHAIRS,
        OSD_ARTIFICIAL_HORIZON,
        OSD_HORIZON_SIDEBARS,
        OSD_ITEM_TIMER_1,
        OSD_ITEM_TIMER_2,
        OSD_FLYMODE,
        OSD_CRAFT_NAME,
        OSD_THROTTLE_POS,
        OSD_VTX_CHANNEL,
        OSD_CURRENT_DRAW,
        OSD_MAH_DRAWN,
        OSD_GPS_SPEED,
        OSD_GPS_SATS,
        OSD_ALTITUDE,
        OSD_ROLL_PIDS,
        OSD_PITCH_PIDS,
        OSD_YAW_PIDS,
        OSD_POWER,
        OSD_PIDRATE_PROFILE,
        OSD_WARNINGS,
        OSD_AVG_CELL_VOLTAGE,
        OSD_GPS_LON,
        OSD_GPS_LAT,
        OSD_DEBUG,
        OSD_PITCH_ANGLE,
        OSD_ROLL_ANGLE,
        OSD_MAIN_BATT_USAGE,
        OSD_DISARMED,
        OSD_HOME_DIR,
        OSD_HOME_DIST,
        OSD_NUMERICAL_HEADING,
        OSD_NUMERICAL_VARIO,
        OSD_COMPASS_BAR,
        OSD_ESC_TMP,
        OSD_ESC_RPM,
        OSD_REMAINING_TIME_ESTIMATE,
        OSD_RTC_DATETIME,
        OSD_ADJUSTMENT_RANGE,
        OSD_CORE_TEMPERATURE,
        OSD_ANTI_GRAVITY,
        OSD_G_FORCE,
        OSD_MOTOR_DIAG,
        OSD_LOG_STATUS,
        OSD_FLIP_ARROW,
        OSD_LINK_QUALITY,
        OSD_FLIGHT_DIST,
        OSD_STICK_OVERLAY_LEFT,
        OSD_STICK_OVERLAY_RIGHT,
        OSD_DISPLAY_NAME,
        OSD_ESC_RPM_FREQ,
        OSD_RATE_PROFILE_NAME,
        OSD_PID_PROFILE_NAME,
        OSD_PROFILE_NAME,
        OSD_RSSI_DBM_VALUE,
        OSD_RC_CHANNELS,
        OSD_CAMERA_FRAME,
        OSD_ITEM_COUNT // MUST BE LAST
    } osd_items_e;

    typedef enum {
        OSD_STAT_RTC_DATE_TIME,
        OSD_STAT_TIMER_1,
        OSD_STAT_TIMER_2,
        OSD_STAT_MAX_SPEED,
        OSD_STAT_MAX_DISTANCE,
        OSD_STAT_MIN_BATTERY,
        OSD_STAT_END_BATTERY,
        OSD_STAT_BATTERY,
        OSD_STAT_MIN_RSSI,
        OSD_STAT_MAX_CURRENT,
        OSD_STAT_USED_MAH,
        OSD_STAT_MAX_ALTITUDE,
        OSD_STAT_BLACKBOX,
        OSD_STAT_BLACKBOX_NUMBER,
        OSD_STAT_MAX_G_FORCE,
        OSD_STAT_MAX_ESC_TEMP,
        OSD_STAT_MAX_ESC_RPM,
        OSD_STAT_MIN_LINK_QUALITY,
        OSD_STAT_FLIGHT_DISTANCE,
        OSD_STAT_MAX_FFT,
        OSD_STAT_TOTAL_FLIGHTS,
        OSD_STAT_TOTAL_TIME,
        OSD_STAT_TOTAL_DIST,
        OSD_STAT_MIN_RSSI_DBM,
        OSD_STAT_COUNT // MUST BE LAST
    } osd_stats_e;

    typedef enum : uint8_t {
        OSD_UNIT_IMPERIAL,
        OSD_UNIT_METRIC
    } osd_unit_e;

    typedef enum {
        OSD_TIMER_1,
        OSD_TIMER_2,
        OSD_TIMER_COUNT
    } osd_timer_e;

    typedef enum {
        OSD_WARNING_ARMING_DISABLE,
        OSD_WARNING_BATTERY_NOT_FULL,
        OSD_WARNING_BATTERY_WARNING,
        OSD_WARNING_BATTERY_CRITICAL,
        OSD_WARNING_VISUAL_BEEPER,
        OSD_WARNING_CRASH_FLIP,
        OSD_WARNING_ESC_FAIL,
        OSD_WARNING_CORE_TEMPERATURE,
        OSD_WARNING_RC_SMOOTHING,
        OSD_WARNING_FAIL_SAFE,
        OSD_WARNING_LAUNCH_CONTROL,
        OSD_WARNING_GPS_RESCUE_UNAVAILABLE,
        OSD_WARNING_GPS_RESCUE_DISABLED,
        OSD_WARNING_RSSI,
        OSD_WARNING_LINK_QUALITY,
        OSD_WARNING_RSSI_DBM,
        OSD_WARNING_COUNT // MUST BE LAST
    } osd_warnings_flags_e;

    typedef struct osd_config_s {
        osd_unit_e units;
        uint8_t rssi_alarm;
        uint16_t cap_alarm;
        uint16_t alt_alarm;
        uint16_t timers[OSD_TIMER_COUNT];
        uint32_t enabled_warnings;
        uint32_t enabled_stats;
    } osd_config_t;
}

#endif  //HAL_MSP_ENABLED