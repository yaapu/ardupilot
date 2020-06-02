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

/*
   MSP Telemetry library
*/

#include <AP_Common/AP_FWVersion.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <RC_Channel/RC_Channel.h>
#include <ctype.h>
#include <stdio.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_Backend.h"

#if  HAL_MSP_ENABLED

extern const AP_HAL::HAL& hal;
constexpr uint8_t AP_MSP_Telem_Backend::arrows[8];

//#define MSP_OSD_POS_HIDDEN 234
#define MSP_OSD_POS_HIDDEN 0

AP_MSP_Telem_Backend::AP_MSP_Telem_Backend(AP_MSP& msp, uint8_t protocol_instance, bool scheduler_enabled) : AP_RCTelemetry(MSP_TIME_SLOT_MAX),
    _protocol_instance(protocol_instance),
    _msp(msp),
    _scheduler_enabled(scheduler_enabled)
{
}

AP_MSP_Telem_Backend::~AP_MSP_Telem_Backend(void)
{
}

void AP_MSP_Telem_Backend::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    set_scheduler_entry(EMPTY_SLOT,50,50);          // nothing to send
    set_scheduler_entry(NAME,200,200);              // 5Hz  12 chars string used for general purpose text messages
    set_scheduler_entry(STATUS,500,500);            // 2Hz  flightmode
    set_scheduler_entry(CONFIG,200,200);            // 5Hz  OSD item positions
    set_scheduler_entry(RAW_GPS,250,250);           // 4Hz  GPS lat/lon
    set_scheduler_entry(COMP_GPS,250,250);          // 4Hz  home direction and distance
    set_scheduler_entry(ATTITUDE,200,200);          // 5Hz  attitude
    set_scheduler_entry(ALTITUDE,250,250);          // 4Hz  altitude(cm) and velocity(cm/s)
    set_scheduler_entry(ANALOG,250,250);            // 4Hz  rssi + batt
    set_scheduler_entry(BATTERY_STATE,500,500);     // 2Hz  battery
    set_scheduler_entry(ESC_SENSOR_DATA,500,500);   // 2Hz  ESC telemetry
    set_scheduler_entry(RTC_DATETIME,1000,1000);    // 1Hz  RTC
}

/*
 * init - perform required initialisation
 */
bool AP_MSP_Telem_Backend::init()
{
    enable_warnings();

    return AP_RCTelemetry::init();
}

void AP_MSP_Telem_Backend::run_wfq_scheduler()
{
    if (!_scheduler_enabled) {
        return;
    }

    AP_RCTelemetry::run_wfq_scheduler();
}

void AP_MSP_Telem_Backend::adjust_packet_weight(bool queue_empty)
{
}

// WFQ scheduler
bool AP_MSP_Telem_Backend::is_packet_ready(uint8_t idx, bool queue_empty)
{
    switch (idx) {
        case EMPTY_SLOT:        // empty slot
            return true;
        case NAME:              // used for status_text messages
            return true;
        case STATUS:            // flightmode
            return true;
        case CONFIG:            // OSD config
            return true;
        case RAW_GPS:           // lat,lon, speed
            return true;
        case COMP_GPS:          // home dir,dist
            return true;
        case ATTITUDE:          // Attitude
            return true;
        case ALTITUDE:          // Altitude and Vario
            return true;
        case ANALOG:            // Rssi, Battery, mAh, Current
            return true;
        case BATTERY_STATE:     // voltage, capacity, current, mAh
            return true;
        case ESC_SENSOR_DATA:   // esc temp + rpm
            return true;
        case RTC_DATETIME:      // RTC
            return true;
        default:
            return false;
    }
}

/*
 * WFQ scheduler
 */
void AP_MSP_Telem_Backend::process_packet(uint8_t idx)
{
    // at each wfq scheduler step invoke a callback on MSP
    // we have a single thread so all access should be safe
    if (idx == EMPTY_SLOT) {
        return;
    }
    _msp.msp_telemetry_callback(msp_packet_type_map[idx],_protocol_instance);
}

bool AP_MSP_Telem_Backend::get_next_msg_chunk(void)
{
    return true;
}

uint8_t AP_MSP_Telem_Backend::calc_cell_count(float battery_voltage)
{
    return floorf((battery_voltage / CELLFULL) + 1);
}

float AP_MSP_Telem_Backend::get_vspeed_ms(void)
{
    {
        // release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v;
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }
    auto &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}

void AP_MSP_Telem_Backend::telem_update_home_pos(telemetry_info_t& _telemetry)
{
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    Location loc;
    float alt;
    if (_ahrs.get_position(loc) && _ahrs.home_is_set()) {
        const Location &home_loc = _ahrs.get_home();
        _telemetry.home_distance_m = home_loc.get_distance(loc);
        _telemetry.home_bearing_cd = loc.get_bearing_to(home_loc);
    } else {
        _telemetry.home_distance_m = 0;
        _telemetry.home_bearing_cd = 0;
    }
    _ahrs.get_relative_position_D_home(alt);
    _telemetry.rel_altitude_cm = -alt * 100;
    _telemetry.vspeed_ms = get_vspeed_ms();
    _telemetry.home_is_set = _ahrs.home_is_set();
}

void AP_MSP_Telem_Backend::telem_update_gps_state(telemetry_info_t& _telemetry)
{
    const AP_GPS& gps = AP::gps();
    _telemetry.gps_fix_type = gps.status();

    if (_telemetry.gps_fix_type >= AP_GPS::GPS_Status::GPS_OK_FIX_2D) {
        _telemetry.gps_num_sats = gps.num_sats();
    } else {
        _telemetry.gps_num_sats = 0;
    }

    if (_telemetry.gps_fix_type >= AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        const Location &loc = AP::gps().location(); //get gps instance 0
        _telemetry.gps_latitude = loc.lat;
        _telemetry.gps_longitude = loc.lng;
        _telemetry.gps_altitude_cm = loc.alt;
        _telemetry.gps_speed_ms = gps.ground_speed();
        _telemetry.gps_ground_course_cd = gps.ground_course_cd();
    } else {
        _telemetry.gps_latitude = 0;
        _telemetry.gps_longitude = 0;
        _telemetry.gps_altitude_cm = 0;
        _telemetry.gps_speed_ms = 0;
        _telemetry.gps_ground_course_cd = 0;
    }
}

void AP_MSP_Telem_Backend::telem_update_battery_state(telemetry_info_t& _telemetry)
{
    const AP_BattMonitor &_battery = AP::battery();
    if (!_battery.current_amps(_telemetry.batt_current_a, 0)) {
        _telemetry.batt_current_a = 0;
    }
    if (!_battery.consumed_mah(_telemetry.batt_consumed_mah, 0)) {
        _telemetry.batt_consumed_mah = 0;
    }
    _telemetry.batt_voltage_v =_battery.voltage(0);
    _telemetry.batt_capacity_mah = _battery.pack_capacity_mah(0);

    const AP_Notify& notify = AP::notify();
    if (notify.flags.failsafe_battery) {
        _telemetry.batt_state = MSP_BATTERY_CRITICAL;
    } else {
        _telemetry.batt_state = MSP_BATTERY_OK;
    }
    // detect cellcount once and accept only higher values, we do not want to update it while discharging
    uint8_t cc = calc_cell_count(_telemetry.batt_voltage_v);
    if (cc > _telemetry.batt_cellcount) {
        _telemetry.batt_cellcount = cc;
    }

}

void AP_MSP_Telem_Backend::telem_update_attitude(telemetry_info_t &_telemetry)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    _telemetry.roll_cd = _ahrs.roll_sensor;             // centidegress to decidegrees -1800,1800
    _telemetry.pitch_cd = _ahrs.pitch_sensor;           // centidegress to decidegrees -1800,1800
    _telemetry.yaw_deg = _ahrs.yaw_sensor * 0.01;       // degrees
}

void AP_MSP_Telem_Backend::telem_update_airspeed(telemetry_info_t &_telemetry)
{
    float aspd = 0.0f;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    _telemetry.airspeed_have_estimate = ahrs.airspeed_estimate(aspd);
    if (_telemetry.airspeed_have_estimate) {
        _telemetry.airspeed_estimate_ms = aspd;
    } else {
        _telemetry.airspeed_estimate_ms = 0.0;
    }
}


void AP_MSP_Telem_Backend::telem_update_flight_mode(telemetry_info_t &_telemetry)
{
    // possible layouts
    // MANU
    // MANU [FS]
    // MANU [SS]
    // MANU [FS|SS]

    const AP_Notify *notify = AP_Notify::get_singleton();

    char buffer[MSP_TXT_BUFFER_SIZE+1] {};
    char* buffer_ptr = buffer;

    bool failsafe = notify->flags.failsafe_battery || notify->flags.failsafe_gcs || notify->flags.failsafe_radio || notify->flags.ekf_bad;
    bool simple_mode = gcs().simple_input_active();
    bool supersimple_mode = gcs().supersimple_input_active();

    uint8_t flags_count = 0;
    // clear
    memset(_telemetry.flight_mode_str,0,MSP_TXT_BUFFER_SIZE+1);
    // first copy flightmode
    strncpy(buffer_ptr, notify->get_flight_mode_str(), 4);
    buffer_ptr += 4;
    // flags open
    strncpy(buffer_ptr, " [",2);
    buffer_ptr += 2;
    // failsafe
    if (failsafe) {
        strncpy(buffer_ptr, "FS|",3);
        buffer_ptr += 3;
        flags_count++;
    }
    // simple mode
    if (simple_mode) {
        strncpy(buffer_ptr, "S",1);
        buffer_ptr += 1;
        flags_count++;
    } else if (supersimple_mode) {
        strncpy(buffer_ptr, "SS",2);
        buffer_ptr += 2;
        flags_count++;
    }

    if (flags_count == 0) {
        memset(buffer+4, 0, 8);         // remove unused []
        buffer_ptr = buffer + 4;
    } else {
        if (failsafe && flags_count == 1) {
            buffer_ptr--;               // remove unused |
        }
        // flags close
        strncpy(buffer_ptr, "]",1);
        buffer_ptr += 1;
    }
    // center
    uint8_t padding = (MSP_TXT_VISIBLE_CHARS - (buffer_ptr - buffer))/2;
    if (padding > 0) {
        memset(_telemetry.flight_mode_str,' ', padding);
    }
    // padding + strlen(buffer) <= MSP_TXT_BUFFER_SIZE
    strncpy(_telemetry.flight_mode_str+padding, buffer, strlen(buffer));
}

void AP_MSP_Telem_Backend::telem_update_localtime(telemetry_info_t &_telemetry)
{
    uint64_t time_usec = 0;
    if (AP::rtc().get_utc_usec(time_usec)) { // may fail, leaving time_unix at 0
        const time_t time_sec = time_usec / 1000000;
        _telemetry.localtime_tm = *gmtime(&time_sec);
    } else {
        _telemetry.localtime_tm.tm_year = 0;
        _telemetry.localtime_tm.tm_mon = 0;
        _telemetry.localtime_tm.tm_mday = 0;
        _telemetry.localtime_tm.tm_hour = 0;
        _telemetry.localtime_tm.tm_min = 0;
        _telemetry.localtime_tm.tm_sec = 0;
    }
}

void AP_MSP_Telem_Backend::telem_update_wind(telemetry_info_t &_telemetry)
{
    const AP_Notify *notify = AP_Notify::get_singleton();
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    Vector3f v = ahrs.wind_estimate();
    /*
    if (check_option(AP_OSD::OPTION_INVERTED_WIND)) {
        v = -v;
    }
    */
    float v_length = v.length();
    // clear
    memset(_telemetry.flight_mode_str,0,MSP_TXT_BUFFER_SIZE+1);
    /*
    if (u_scale(SPEED, v_length) < 10.0) {
        backend->write(x, y, false, "%c%3.1f%c", arrow, u_scale(SPEED, v_length), u_icon(SPEED));
    } else {
        backend->write(x, y, false, "%c%3d%c", arrow, (int)u_scale(SPEED, v_length), u_icon(SPEED));
    }
    */
    if (v_length > 1.0f) {
        int32_t angle = wrap_360_cd(DEGX100 * atan2f(v.y, v.x) - ahrs.yaw_sensor);
        int32_t interval = 36000 / MSP_ARROW_COUNT;
        uint8_t arrow = arrows[((angle + interval / 2) / interval) % MSP_ARROW_COUNT];

        // flightmode + wind can use up to MSP_TXT_VISIBLE_CHARS-1 chars, leaving 1 visible char for a multibyte utf8 arrow
        // example: MANU 4m/s ↗ 
        int8_t len = snprintf(_telemetry.flight_mode_str, MSP_TXT_VISIBLE_CHARS-1, "%s %dm/s ", notify->get_flight_mode_str(),  (uint8_t)roundf(v_length));

        // UTF8 arrow bytes 0xE286nn
        _telemetry.flight_mode_str[len] = 0xE2;
        _telemetry.flight_mode_str[len+1] = 0x86;
        _telemetry.flight_mode_str[len+2] = arrow;
    } else {
        // no more than MSP_TXT_VISIBLE_CHARS chars
        snprintf(_telemetry.flight_mode_str, MSP_TXT_VISIBLE_CHARS, "%s ---m/s", notify->get_flight_mode_str());
    }
}


void AP_MSP_Telem_Backend::enable_warnings()
{
    BIT_SET(_msp.osd_config.enabled_warnings, OSD_WARNING_FAIL_SAFE);
    BIT_SET(_msp.osd_config.enabled_warnings, OSD_WARNING_BATTERY_CRITICAL);
}

bool AP_MSP_Telem_Backend::msp_process_out_raw_gps(sbuf_t *dst)
{
    telem_update_gps_state(_msp.telemetry_info);

    sbuf_write_u8(dst, _msp.telemetry_info.gps_fix_type >= 3 ? 2 : 0);   // bitmask 1 << 1 is GPS FIX
    sbuf_write_u8(dst, _msp.telemetry_info.gps_num_sats);
    sbuf_write_u32(dst, _msp.telemetry_info.gps_latitude);
    sbuf_write_u32(dst, _msp.telemetry_info.gps_longitude);
    sbuf_write_u16(dst, (uint16_t)constrain_int32(_msp.telemetry_info.gps_altitude_cm / 100, 0, UINT16_MAX));   // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
    // handle airspeed override
    if ( _msp.airspeed_en) {
        telem_update_airspeed(_msp.telemetry_info);
        sbuf_write_u16(dst, _msp.telemetry_info.airspeed_estimate_ms * 100);                // airspeed in cm/s
    } else {
        sbuf_write_u16(dst, (uint16_t)roundf(_msp.telemetry_info.gps_speed_ms * 100));      // speed in cm/s
    }
    sbuf_write_u16(dst, _msp.telemetry_info.gps_ground_course_cd * 1000);                   // degrees * 10 == centideg * 1000
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_comp_gps(sbuf_t *dst)
{
    telem_update_home_pos(_msp.telemetry_info);
    telem_update_attitude(_msp.telemetry_info);

    // no need to apply yaw conpensation, the DJI air unit will do it for us :-)
    int32_t home_angle_deg = _msp.telemetry_info.home_bearing_cd * 0.01;
    if (_msp.telemetry_info.home_distance_m < 2) {
        //avoid fast rotating arrow at small distances
        home_angle_deg = 0;
    }

    sbuf_write_u16(dst, _msp.telemetry_info.home_distance_m);
    sbuf_write_u16(dst, (uint16_t)home_angle_deg); //deg
    sbuf_write_u8(dst, 1); // 1 is toggle GPS position update
    return true;
}

// Autoscroll message is the same as in minimosd-extra.
// Thanks to night-ghost for the approach.
bool AP_MSP_Telem_Backend::msp_process_out_name(sbuf_t *dst)
{
    AP_Notify * notify = AP_Notify::get_singleton();
    if (notify) {
        // text message is visible for _msp.msgtime_s but only if
        // a flight mode change did not steal focus
        int32_t visible_time = AP_HAL::millis() - notify->get_text_updated_millis();
        if (visible_time < _msp.msgtime_s *1000 && !_msp.flight_mode_focus) {
            char buffer[NOTIFY_TEXT_BUFFER_SIZE];
            strncpy(buffer, notify->get_text(), sizeof(buffer));
            uint8_t len = strnlen(buffer, sizeof(buffer));

            for (uint8_t i=0; i<len; i++) {
                //converted to uppercase,
                //because we do not have small letter chars inside used font
                buffer[i] = toupper(buffer[i]);
                //normalize whitespace
                if (isspace(buffer[i])) {
                    buffer[i] = ' ';
                }
            }

            int8_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > MSP_TXT_VISIBLE_CHARS) {
                uint8_t chars_to_scroll = len - MSP_TXT_VISIBLE_CHARS;
                uint8_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                uint8_t current_cycle = (visible_time / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                uint8_t end_position = start_position + MSP_TXT_VISIBLE_CHARS;

                //ensure array boundaries
                start_position = MIN(start_position, int(sizeof(buffer)-1));
                end_position = MIN(end_position, int(sizeof(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }

            sbuf_write_data(dst, buffer + start_position, strlen(buffer + start_position));  // max MSP_TXT_VISIBLE_CHARS chars general text...
        } else {
            if (_msp.wind_en > 0) {
                telem_update_wind(_msp.telemetry_info);
            } else {
                telem_update_flight_mode(_msp.telemetry_info);
            }
            sbuf_write_data(dst, _msp.telemetry_info.flight_mode_str, MSP_TXT_BUFFER_SIZE);  // rendered as up to MSP_TXT_VISIBLE_CHARS chars with UTF8 support
        }
    }
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_status(sbuf_t *dst)
{
    uint32_t mode_bitmask = get_osd_flight_mode_bitmask();
    sbuf_write_u16(dst, 0);                     // task delta time
    sbuf_write_u16(dst, 0);                     // I2C error count
    sbuf_write_u16(dst, 0);                     // sensor status
    sbuf_write_data(dst, &mode_bitmask, 4);     // unconditional part of flags, first 32 bits
    sbuf_write_u8(dst, 0);

    sbuf_write_u16(dst, constrain_int16(0, 0, 100));  //system load
    sbuf_write_u16(dst, 0);                     // gyro cycle time

    // Cap BoxModeFlags to 32 bits
    sbuf_write_u8(dst, 0);

    // Write arming disable flags
    sbuf_write_u8(dst, 1);
    sbuf_write_u32(dst, !AP::notify().flags.armed);

    // Extra flags
    sbuf_write_u8(dst, 0);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_osd_config(sbuf_t *dst)
{
    sbuf_write_u8(dst, OSD_FLAGS_OSD_FEATURE);                      // flags
    sbuf_write_u8(dst, 0);                                          // video system
    // Configuration
    sbuf_write_u8(dst, _msp.units);                                 // units
    // Alarms
    sbuf_write_u8(dst, _msp.osd_config.rssi_alarm);                 // rssi alarm
    sbuf_write_u16(dst, _msp.osd_config.cap_alarm);                 // capacity alarm
    // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, OSD_ITEM_COUNT);                             // osd items count

    sbuf_write_u16(dst, _msp.osd_config.alt_alarm);                 // altitude alarm

    // element position and visibility
    uint16_t pos = 0;
    for (uint8_t i = 0; i < OSD_ITEM_COUNT; i++) {
        pos = MSP_OSD_POS_HIDDEN;
        if (_msp.osd_item_settings[i] != nullptr) {      // ok supported
            if (_msp.osd_item_settings[i]->enabled) {    // ok enabled
                // let's check if we need to hide this dynamically
                if (!BIT_IS_SET(osd_hidden_items_bitmask, i)) {
                    pos = MSP_OSD_POS(_msp.osd_item_settings[i]);
                }
            }
        }
        sbuf_write_u16(dst, pos);
    }

    // post flight statistics
    sbuf_write_u8(dst, OSD_STAT_COUNT);                         // stats items count
    for (uint8_t i = 0; i < OSD_STAT_COUNT; i++ ) {
        if ( _msp.osd_enabled_stats[i] >= 0) {
            // no stats support yet
            sbuf_write_u16(dst, 0);
        } else {
            // hide this OSD element
            sbuf_write_u16(dst, 0);
        }
    }

    // timers
    sbuf_write_u8(dst, OSD_TIMER_COUNT);                      // timers
    for (uint8_t i = 0; i < OSD_TIMER_COUNT; i++) {
        // no timer support
        sbuf_write_u16(dst, 0);
    }

    // Enabled warnings
    // API < 1.41
    // Send low word first for backwards compatibility
    sbuf_write_u16(dst, (uint16_t)(_msp.osd_config.enabled_warnings & 0xFFFF)); // Enabled warnings
    // API >= 1.41
    // Send the warnings count and 32bit enabled warnings flags.
    // Add currently active OSD profile (0 indicates OSD profiles not available).
    // Add OSD stick overlay mode (0 indicates OSD stick overlay not available).
    sbuf_write_u8(dst, OSD_WARNING_COUNT);            // warning count
    sbuf_write_u32(dst, _msp.osd_config.enabled_warnings);  // enabled warning

    // If the feature is not available there is only 1 profile and it's always selected
    sbuf_write_u8(dst, 1);    // available profiles
    sbuf_write_u8(dst, 1);    // selected profile

    sbuf_write_u8(dst, 0);    // OSD stick overlay

    // API >= 1.43
    // Add the camera frame element width/height
    //sbuf_write_u8(dst, osdConfig()->camera_frame_width);
    //sbuf_write_u8(dst, osdConfig()->camera_frame_height);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_attitude(sbuf_t *dst)
{
    telem_update_attitude(_msp.telemetry_info);

    sbuf_write_u16(dst, (int16_t)(_msp.telemetry_info.roll_cd * 0.1));                // centidegress to decidegrees -1800,1800
    sbuf_write_u16(dst, (int16_t)(_msp.telemetry_info.pitch_cd * 0.1));               // centidegress to decidegrees -1800,1800
    sbuf_write_u16(dst, (int16_t)_msp.telemetry_info.yaw_deg);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_altitude(sbuf_t *dst)
{
    telem_update_home_pos(_msp.telemetry_info);

    sbuf_write_u32(dst, _msp.telemetry_info.rel_altitude_cm);                   // relative altitude cm
    sbuf_write_u16(dst, (int16_t)_msp.telemetry_info.vspeed_ms * 100);          // climb rate cm/s
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_analog(sbuf_t *dst)
{
    telem_update_battery_state(_msp.telemetry_info);
    AP_RSSI* _rssi = AP::rssi();

    sbuf_write_u8(dst,  (uint8_t)constrain_int16(_msp.telemetry_info.batt_voltage_v * 10, 0, 255));     // battery voltage V to dV
    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_consumed_mah, 0, 0xFFFF));             // milliamp hours drawn from battery
    sbuf_write_u16(dst, _rssi->enabled() ? _rssi->read_receiver_rssi() * 1023 : 0);                     // rssi 0-1 to 0-1023
    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_current_a * 100, -0x8000, 0x7FFF));    // current A to cA (0.01 steps, range is -320A to 320A)
    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_voltage_v * 100,0,0xFFFF));            // battery voltage in 0.01V steps

    return true;
}


bool AP_MSP_Telem_Backend::msp_process_out_battery_state(sbuf_t *dst)
{
    telem_update_battery_state(_msp.telemetry_info);

    // battery characteristics
    sbuf_write_u8(dst, (uint8_t)constrain_int16((_msp.cellcount > 0 ? _msp.cellcount : _msp.telemetry_info.batt_cellcount), 0, 255)); // cell count 0 indicates battery not detected.
    sbuf_write_u16(dst, _msp.telemetry_info.batt_capacity_mah); // in mAh

    // battery state
    sbuf_write_u8(dst,  (uint8_t)constrain_int16(_msp.telemetry_info.batt_voltage_v * 10, 0, 255));    // battery voltage V to dV
    sbuf_write_u16(dst, (uint16_t)MIN(_msp.telemetry_info.batt_consumed_mah, 0xFFFF));                 // milliamp hours drawn from battery
    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_current_a * 100, -0x8000, 0x7FFF));   // current A to cA (0.01 steps, range is -320A to 320A)

    // battery alerts
    sbuf_write_u8(dst, _msp.telemetry_info.batt_state);  // BATTERY: OK=0, CRITICAL=2

    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_voltage_v * 100, 0, 0x7FFF));         // battery voltage in 0.01V steps
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_esc_sensor_data(sbuf_t *dst)
{
#ifdef HAVE_AP_BLHELI_SUPPORT
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (blheli) {
        AP_BLHeli::telem_data td;
        sbuf_write_u8(dst, blheli->get_num_motors());
        for (uint8_t i = 0; i < blheli->get_num_motors(); i++) {
            if (blheli->get_telem_data(i, td)) {
                sbuf_write_u8(dst, td.temperature);        // deg
                sbuf_write_u16(dst, td.rpm * 0.01);        // eRpm to 0.01 eRpm
            }
        }
    }
#endif
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_rtc(sbuf_t *dst)
{
    telem_update_localtime(_msp.telemetry_info);

    sbuf_write_u16(dst, _msp.telemetry_info.localtime_tm.tm_year + 1900);   // tm_year is relative to year 1900
    sbuf_write_u8(dst, _msp.telemetry_info.localtime_tm.tm_mon + 1);        // MSP requires [1-12] months
    sbuf_write_u8(dst, _msp.telemetry_info.localtime_tm.tm_mday);
    sbuf_write_u8(dst, _msp.telemetry_info.localtime_tm.tm_hour);
    sbuf_write_u8(dst, _msp.telemetry_info.localtime_tm.tm_min);
    sbuf_write_u8(dst, _msp.telemetry_info.localtime_tm.tm_sec);
    sbuf_write_u16(dst, 0);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_rc(sbuf_t *dst)
{
    RCMapper* rcmap = AP::rcmap();
    uint16_t values[16] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));

    // send only 4 channels, MSP order is AERT
    sbuf_write_u16(dst, values[rcmap->roll()]);     // A
    sbuf_write_u16(dst, values[rcmap->pitch()]);    // E
    sbuf_write_u16(dst, values[rcmap->yaw()]);      // R
    sbuf_write_u16(dst, values[rcmap->throttle()]); // T

    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_board_info(sbuf_t *dst)
{
    const AP_FWVersion &fwver = AP::fwversion();

    sbuf_write_data(dst, "ARDU", BOARD_IDENTIFIER_LENGTH);
    sbuf_write_u16(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, strlen(fwver.fw_string));
    sbuf_write_data(dst, fwver.fw_string, strlen(fwver.fw_string));
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_build_info(sbuf_t *dst)
{
    const AP_FWVersion &fwver = AP::fwversion();

    sbuf_write_data(dst, __DATE__, BUILD_DATE_LENGTH);
    sbuf_write_data(dst, __TIME__, BUILD_TIME_LENGTH);
    sbuf_write_data(dst, fwver.fw_hash_str, GIT_SHORT_REVISION_LENGTH);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_uid(sbuf_t *dst)
{
    sbuf_write_u32(dst, 0xAABBCCDD);
    sbuf_write_u32(dst, 0xAABBCCDD);
    sbuf_write_u32(dst, 0xAABBCCDD);
    return true;
}

void AP_MSP_Telem_Backend::flash_osd_items(void)
{
    const AP_Notify *notify = AP_Notify::get_singleton();
    // flash satcount when no 3D Fix
    if (_msp.telemetry_info.gps_fix_type < AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        if (_msp.flashing_on) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SATS);
        } else {
            BIT_SET(osd_hidden_items_bitmask, OSD_GPS_SATS);
        }
    } else {
        BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SATS);
    }
    // flash home dir and distance if home is not set
    if (!_msp.telemetry_info.home_is_set) {
        if (_msp.flashing_on) {
            BIT_SET(osd_hidden_items_bitmask, OSD_HOME_DIR);
            BIT_SET(osd_hidden_items_bitmask, OSD_HOME_DIST);
        } else {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIR);
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIST);
        }
    } else {
        BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIR);
        BIT_CLEAR(osd_hidden_items_bitmask, OSD_HOME_DIST);
    }
    // flash airspeed if there's no estimate
    if (_msp.airspeed_en) {
        if (!_msp.telemetry_info.airspeed_have_estimate) {
            if (_msp.flashing_on) {
                BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SPEED);
            } else {
                BIT_SET(osd_hidden_items_bitmask, OSD_GPS_SPEED);
            }
        } else {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_GPS_SPEED);
        }
    }
    // flash flightmode on failsafe (even if messages are scrolling?)
    if (notify->flags.failsafe_battery || notify->flags.failsafe_gcs || notify->flags.failsafe_radio || notify->flags.ekf_bad) {
        if (_msp.flashing_on) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        } else {
            BIT_SET(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        }
    } else {
        // flash text flightmode for 3secs after each change
        if (_msp.flight_mode_focus) {
            if (_msp.flashing_on) {
                BIT_CLEAR(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
            } else {
                BIT_SET(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
            }
        } else {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        }
    }
    // flash battery on failsafe
    if (notify->flags.failsafe_battery) {
        if (_msp.flashing_on) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_AVG_CELL_VOLTAGE);
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_MAIN_BATT_VOLTAGE);
        } else {
            BIT_SET(osd_hidden_items_bitmask, OSD_AVG_CELL_VOLTAGE);
            BIT_SET(osd_hidden_items_bitmask, OSD_MAIN_BATT_VOLTAGE);
        }
    }
    // flash rtc if no time available
    if (_msp.telemetry_info.localtime_tm.tm_year == 0) {
        if (_msp.flashing_on) {
            BIT_CLEAR(osd_hidden_items_bitmask, OSD_RTC_DATETIME);
        } else {
            BIT_SET(osd_hidden_items_bitmask, OSD_RTC_DATETIME);
        }
    }
}

void AP_MSP_Telem_Backend::apply_osd_items_overrides(void)
{
}

#endif  //HAL_MSP_ENABLED