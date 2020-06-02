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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_Backend.h"

#if  HAL_MSP_ENABLED

extern const AP_HAL::HAL& hal;


AP_MSP_Telem_Backend::AP_MSP_Telem_Backend(AP_MSP& msp, uint8_t protocol_instance, bool scheduler_enabled) : AP_RCTelemetry(MSP_TIME_SLOT_MAX),
    _protocol_instance(protocol_instance),
    _msp(msp),
    _scheduler_enabled(scheduler_enabled)
{
}

AP_MSP_Telem_Backend::~AP_MSP_Telem_Backend(void)
{
}

/*
  setup ready for passthrough telem
 */
void AP_MSP_Telem_Backend::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )

    set_scheduler_entry(EMPTY_SLOT,50,50);          // nothing to send
    set_scheduler_entry(NAME,100,100);               // 10Hz  12 chars string used for general purpose text messages
    set_scheduler_entry(STATUS,500,500);            // 2Hz  flightmode
    set_scheduler_entry(CONFIG,200,200);            // 5Hz  OSD item positions
    set_scheduler_entry(RAW_GPS,250,250);           // 4Hz  GPS lat/lon
    set_scheduler_entry(COMP_GPS,250,250);          // 4Hz  home direction and distance
    set_scheduler_entry(ATTITUDE,100,100);          // 10Hz attitude
    set_scheduler_entry(ALTITUDE,250,250);          // 4Hz  altitude(cm) and velocity(cm/s)
    set_scheduler_entry(ANALOG,250,250);            // 4Hz  rssi + batt
    set_scheduler_entry(BATTERY_STATE,500,500);     // 2Hz  battery
}

/*
 * init - perform required initialisation
 */
bool AP_MSP_Telem_Backend::init()
{
    return AP_RCTelemetry::init();
}

void AP_MSP_Telem_Backend::run_wfq_scheduler() {
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
        case EMPTY_SLOT:
            return true;
        case NAME:          // used for status_text messages
            return true;
        case STATUS:        // flightmode
            return true;
        case CONFIG:        // OSD config
            return true;
        case RAW_GPS:       // lat,lon, speed
            return true;
        case COMP_GPS:      // home dir,dist
            return true;
        case ATTITUDE:      // Attitude
            return true;
        case ALTITUDE:      // Altitude and Vario
            return true;
        case ANALOG:        // Rssi, Battery, mAh, Current
            return true;
        case BATTERY_STATE: // voltage, capacity, current, mAh
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
    {// release semaphore as soon as possible
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
    _telemetry.gps_num_sats = gps.num_sats();
    if (_telemetry.gps_fix_type >= 3) {
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

void AP_MSP_Telem_Backend::telem_update_battery_state(telemetry_info_t& _telemetry) {
    const AP_BattMonitor &_battery = AP::battery();
    if (!_battery.current_amps(_telemetry.batt_current_a, 0)) {
        _telemetry.batt_current_a = 0;
    }
    if (!_battery.consumed_mah(_telemetry.batt_consumed_mah, 0)) {
        _telemetry.batt_consumed_mah = 0;
    }
    _telemetry.batt_voltage_v =_battery.voltage(0); 
    _telemetry.batt_capacity_mah = _battery.pack_capacity_mah(0);
}

void AP_MSP_Telem_Backend::telem_update_attitude(telemetry_info_t &_telemetry) {
    const AP_AHRS &_ahrs = AP::ahrs();
    
    _telemetry.roll_cd = _ahrs.roll_sensor;             // centidegress to decidegrees -1800,1800
    _telemetry.pitch_cd = _ahrs.pitch_sensor;           // centidegress to decidegrees -1800,1800
    _telemetry.yaw_deg = _ahrs.yaw_sensor * 0.01;              // degrees
}

void AP_MSP_Telem_Backend::telem_update_airspeed(telemetry_info_t &_telemetry) {
    float aspd = 0.0f;
    AP_AHRS &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    _telemetry.airspeed_have_estimate = ahrs.airspeed_estimate(aspd);
    if (_telemetry.airspeed_have_estimate) {
        _telemetry.airspeed_estimate_cms = aspd;
    } else {
        _telemetry.airspeed_estimate_cms = 0.0;
    }
}
uint32_t AP_MSP_Telem_Backend::get_osd_flight_mode_bitmask(void) 
{
    // default is hide the flightmode
    SET_BIT(osd_hidden_items_bitmask, OSD_FLYMODE);

    uint32_t mode_mask = 0;
    const AP_FWVersion &fwver = AP::fwversion();

    if (strstr(fwver.fw_string,"Plane") != nullptr) {
        switch (AP::notify().flags.flight_mode) {
            case 0:     //MANUAL
            case 1:     //CIRCLE
            case 2:     //STABILIZE
            case 3:     //TRAINING:
            case 4:     //ACRO:
            case 5:     //FLY_BY_WIRE_A:
            case 6:     //FLY_BY_WIRE_B:
            case 7:     //CRUISE:
            case 8:     //AUTOTUNE:
            case 10:    //AUTO:
            case 11:    //RTL:
            case 12:    //LOITER:
            case 13:    //TAKEOFF:
            case 14:    //AVOID_ADSB:
            case 15:    //GUIDED:
            case 16:    //INITIALISING:
            case 17:    //QSTABILIZE:
            case 18:    //QHOVER:
            case 19:    //QLOITER:
            case 20:    //QLAND:
            case 21:    //QRTL:
            case 22:    //QAUTOTUNE:
            case 23:    //QACRO:
                break;
        }
    } else if (strstr(fwver.fw_string,"Copter") != nullptr) {
        switch (AP::notify().flags.flight_mode) {
            case 0:     //STABILIZE =     0,  // manual airframe angle with manual throttle
            case 1:     //ACRO =          1,  // manual body-frame angular rate with manual throttle
            case 2:     //ALT_HOLD =      2,  // manual airframe angle with automatic throttle
            case 3:     //AUTO =          3,  // fully automatic waypoint control using mission commands
            case 4:     //GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
            case 5:     //LOITER =        5,  // automatic horizontal acceleration with automatic throttle
            case 6:     //RTL =           6,  // automatic return to launching point
            case 7:     //CIRCLE =        7,  // automatic circular flight with automatic throttle
            case 9:     //LAND =          9,  // automatic landing with horizontal position control
            case 11:    //DRIFT =        11,  // semi-automous position, yaw and throttle control
            case 13:    //SPORT =        13,  // manual earth-frame angular rate control with manual throttle
            case 14:    //FLIP =         14,  // automatically flip the vehicle on the roll axis
            case 15:    //AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
            case 16:    //POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
            case 17:    //BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
            case 18:    //THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
            case 19:    //AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
            case 20:    //GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
            case 21:    //SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
            case 22:    //FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
            case 23:    //FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
            case 24:    //ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
            case 25:    //SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
            case 26:    //AUTOROTATE =   26,  // Autonomous autorotation
                break;
        }
    }
    return mode_mask;
}

bool AP_MSP_Telem_Backend::msp_process_out_raw_gps(sbuf_t *dst) 
{
    telem_update_gps_state(_msp.telemetry_info);

    sbuf_write_u8(dst, _msp.telemetry_info.gps_fix_type >= 3 ? 2 : 0);   // bitmask 1 << 1 is GPS FIX
    sbuf_write_u8(dst, _msp.telemetry_info.gps_num_sats);
    sbuf_write_u32(dst, _msp.telemetry_info.gps_latitude);
    sbuf_write_u32(dst, _msp.telemetry_info.gps_longitude);
    sbuf_write_u16(dst, (uint16_t)constrain_int32(_msp.telemetry_info.gps_altitude_cm / 100, 0, UINT16_MAX)); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
    sbuf_write_u16(dst, (uint16_t)roundf(_msp.telemetry_info.gps_speed_ms * 10));              // speed in 0.1 m/s
    sbuf_write_u16(dst, _msp.telemetry_info.gps_ground_course_cd * 1000);    // degrees * 10 == centideg * 1000
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_comp_gps(sbuf_t *dst) 
{
    telem_update_home_pos(_msp.telemetry_info);
    telem_update_attitude(_msp.telemetry_info);

    int32_t home_angle_deg = wrap_360(_msp.telemetry_info.home_bearing_cd * 0.01 - _msp.telemetry_info.yaw_deg);
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
// We have 12 chars max
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
            int16_t len = strnlen(buffer, sizeof(buffer));

            for (int16_t i=0; i<len; i++) {
                //converted to uppercase,
                //because we do not have small letter chars inside used font
                buffer[i] = toupper(buffer[i]);
                //normalize whitespace
                if (isspace(buffer[i])) {
                    buffer[i] = ' ';
                }
            }

            int16_t start_position = 0;
            //scroll if required
            //scroll pattern: wait, scroll to the left, wait, scroll to the right
            if (len > message_visible_width) {
                int16_t chars_to_scroll = len - message_visible_width;
                int16_t total_cycles = 2*message_scroll_delay + 2*chars_to_scroll;
                int16_t current_cycle = (visible_time / message_scroll_time_ms) % total_cycles;

                //calculate scroll start_position
                if (current_cycle < total_cycles/2) {
                    //move to the left
                    start_position = current_cycle - message_scroll_delay;
                } else {
                    //move to the right
                    start_position = total_cycles - current_cycle;
                }
                start_position = constrain_int16(start_position, 0, chars_to_scroll);
                int16_t end_position = start_position + message_visible_width;

                //ensure array boundaries
                start_position = MIN(start_position, int(sizeof(buffer)-1));
                end_position = MIN(end_position, int(sizeof(buffer)-1));

                //trim invisible part
                buffer[end_position] = 0;
            }
            
            sbuf_write_data(dst, buffer + start_position, strlen(buffer + start_position));  // max 12 chars general text...
        } else {
            //center 4 chars in 12
            char buffer[9];
            memset(buffer,' ',4);
            strncpy(buffer+4,notify->get_flight_mode_str(),4);
            sbuf_write_data(dst, buffer, 8);
        }
    }
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_status(sbuf_t *dst)
{
    uint32_t mode_bitmask = AP_MSP_Telem_Backend::get_osd_flight_mode_bitmask();
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
    sbuf_write_u8(dst, OSD_FLAGS_OSD_FEATURE);                  // flags
    sbuf_write_u8(dst, 0);                                      // video system
    // Configuration
    sbuf_write_u8(dst, _msp.osd_config.units);                       // units
    // Alarms
    sbuf_write_u8(dst, _msp.osd_config.rssi_alarm);                  // rssi alarm
    sbuf_write_u16(dst, _msp.osd_config.cap_alarm);                  // capacity alarm
    // Reuse old timer alarm (U16) as OSD_ITEM_COUNT
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, OSD_ITEM_COUNT);                         // osd items count

    sbuf_write_u16(dst, _msp.osd_config.alt_alarm);                  // altitude alarm

    // element position and visibility
    uint16_t pos = 0;
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        if (_msp.osd_item_settings[i] != nullptr) {      // ok supported
            if (_msp.osd_item_settings[i]->enabled) {    // ok enabled
                // let's check if we need to hide this dynamically
                if (!BIT_IS_SET(osd_hidden_items_bitmask, i)) {
                    pos = MSP_OSD_POS(_msp.osd_item_settings[i]);
                } else {
                    pos = MSP_OSD_POS_HIDDEN;
                }
            }
        } else {
            pos = MSP_OSD_POS_HIDDEN;
        }
        sbuf_write_u16(dst, pos);
    }

    // post flight statistics
    sbuf_write_u8(dst, OSD_STAT_COUNT);                         // stats items count
    for (int i = 0; i < OSD_STAT_COUNT; i++ ) {
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
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
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

    sbuf_write_u16(dst, (int16_t)(_msp.telemetry_info.roll_cd * 0.1));                  // centidegress to decidegrees -1800,1800
    // handle airspeed override
    if ( _msp.aspd.enabled) { 
        telem_update_airspeed(_msp.telemetry_info);
        sbuf_write_u16(dst, (int16_t)(_msp.telemetry_info.airspeed_estimate_cms * 100));  // airspeed in dm/s
    } else {
        sbuf_write_u16(dst, (int16_t)(_msp.telemetry_info.pitch_cd * 0.1));              // centidegress to decidegrees -1800,1800
    }
    sbuf_write_u16(dst, (int16_t)_msp.telemetry_info.yaw_deg);
    //gcs().send_text(MAV_SEVERITY_DEBUG,"aspd est: %d",_msp.telemetry_info.airspeed_have_estimate);
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_altitude(sbuf_t *dst)
{
    telem_update_home_pos(_msp.telemetry_info);

    sbuf_write_u32(dst, _msp.telemetry_info.rel_altitude_cm);                   // relative altitude cm, negative values are not supported
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


bool AP_MSP_Telem_Backend::msp_process_out_battery_state(sbuf_t *dst){
    telem_update_battery_state(_msp.telemetry_info);

    // battery characteristics
    sbuf_write_u8(dst, (uint8_t)constrain_int16(calc_cell_count(_msp.telemetry_info.batt_voltage_v), 0, 255)); // cell count 0 indicates battery not detected.
    sbuf_write_u16(dst, _msp.telemetry_info.batt_capacity_mah); // in mAh

    // battery state
    sbuf_write_u8(dst,  (uint8_t)constrain_int16(_msp.telemetry_info.batt_voltage_v * 10, 0, 255));    // battery voltage V to dV
    sbuf_write_u16(dst, (uint16_t)MIN(_msp.telemetry_info.batt_consumed_mah, 0xFFFF));                 // milliamp hours drawn from battery
    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_current_a * 100, -0x8000, 0x7FFF));   // current A to cA (0.01 steps, range is -320A to 320A)

    // battery alerts
    sbuf_write_u8(dst, 0);  // BATTERY: OK=0, CRITICAL=2

    sbuf_write_u16(dst, constrain_int16(_msp.telemetry_info.batt_voltage_v * 100, 0, 0x7FFF));         // battery voltage in 0.01V steps
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_board_info(sbuf_t *dst)
{
    sbuf_write_data(dst, "F405", BOARD_IDENTIFIER_LENGTH);
    sbuf_write_u16(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, 0);
    sbuf_write_u8(dst, strlen("Ardupilot"));
    sbuf_write_data(dst, "Ardupilot", strlen("Ardupilot"));
    return true;
}

bool AP_MSP_Telem_Backend::msp_process_out_build_info(sbuf_t *dst)
{
    sbuf_write_data(dst, __DATE__, BUILD_DATE_LENGTH);
    sbuf_write_data(dst, __TIME__, BUILD_TIME_LENGTH);
    sbuf_write_data(dst, "01234567", GIT_SHORT_REVISION_LENGTH);
    return true;
}

void AP_MSP_Telem_Backend::blink_osd_items(void) {
    // blink satcount when no GPS fix
    if (_msp.telemetry_info.gps_fix_type < 3) {
        if (_msp.blink_on) {
            CLEAR_BIT(osd_hidden_items_bitmask, OSD_GPS_SATS);
        } else {
            SET_BIT(osd_hidden_items_bitmask, OSD_GPS_SATS);
        }
    } else {
        CLEAR_BIT(osd_hidden_items_bitmask, OSD_GPS_SATS);
    }
    // blink home dir and distance if home is not set
    if (!_msp.telemetry_info.home_is_set) {
        if (_msp.blink_on) {
            SET_BIT(osd_hidden_items_bitmask, OSD_HOME_DIR);
            SET_BIT(osd_hidden_items_bitmask, OSD_HOME_DIST);
        } else {
            CLEAR_BIT(osd_hidden_items_bitmask, OSD_HOME_DIR);
            CLEAR_BIT(osd_hidden_items_bitmask, OSD_HOME_DIST);
        }
    } else {
        CLEAR_BIT(osd_hidden_items_bitmask, OSD_HOME_DIR);
        CLEAR_BIT(osd_hidden_items_bitmask, OSD_HOME_DIST);
    }
    // blink airspeed is there's no estimate
    if (_msp.aspd.enabled) {
        if (!_msp.telemetry_info.airspeed_have_estimate) {
            if (_msp.blink_on) {
                CLEAR_BIT(osd_hidden_items_bitmask, OSD_PITCH_ANGLE);
            } else {
                SET_BIT(osd_hidden_items_bitmask, OSD_PITCH_ANGLE);
            }
        } else {
            CLEAR_BIT(osd_hidden_items_bitmask, OSD_PITCH_ANGLE);
        }
    }
    // blink text flightmode for 3secs after each change
    if (_msp.flight_mode_focus) {
        if (_msp.blink_on) {
            CLEAR_BIT(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        } else {
            SET_BIT(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
        }
    } else {
        CLEAR_BIT(osd_hidden_items_bitmask, OSD_CRAFT_NAME);
    }
}

void AP_MSP_Telem_Backend::apply_osd_items_overrides(void) 
{
    // override pitch position with airspeed position
    if ( _msp.aspd.enabled) {
        _msp.osd_item_settings[OSD_PITCH_ANGLE] = &_msp.aspd;
    } else {
        _msp.osd_item_settings[OSD_PITCH_ANGLE] = &_msp.pitch;
    }
}

#endif  //HAL_MSP_ENABLED