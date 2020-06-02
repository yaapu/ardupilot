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

#include <AP_Common/AP_FWVersion.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_DJI.h"

#if HAL_MSP_ENABLED
extern const AP_HAL::HAL& hal;

AP_MSP_Telem_DJI::AP_MSP_Telem_DJI(AP_MSP& msp, uint8_t protocol_instance, bool scheduler_enabled) : AP_MSP_Telem_Backend(msp, protocol_instance, scheduler_enabled)
{
}

AP_MSP_Telem_DJI::~AP_MSP_Telem_DJI(void)
{
}

uint32_t AP_MSP_Telem_DJI::get_osd_flight_mode_bitmask(void) 
{
    uint32_t mode_mask = 0;
    // default is hide the DJI flightmode widget
    BIT_SET(osd_hidden_items_bitmask, OSD_FLYMODE);
    const AP_Notify& _notify = AP::notify();
    bool force_visible;    

    // set arming status
    if (_notify.flags.armed) {
        BIT_SET(mode_mask, DJI_MODE_ARM);
    } else {
        // when disarmed no need to show the DJI flight mode
        return mode_mask;
    }
    // check failsafe
    if (_notify.flags.failsafe_battery || _notify.flags.failsafe_gcs || _notify.flags.failsafe_radio) {
        BIT_SET(mode_mask, DJI_MODE_FS);
        force_visible = true;
    }
    const AP_FWVersion &fwver = AP::fwversion();

    if (strstr(fwver.fw_string,"Plane") != nullptr) {
        switch (AP::notify().flags.flight_mode) {
            case 11:    //RTL:
                // show item on RTL
                BIT_SET(mode_mask, DJI_MODE_RESC);
                force_visible = true;
                break;
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
            case 6:     //RTL =           6,  // automatic return to launching point
                // show item on RTL
                BIT_SET(mode_mask, DJI_MODE_RESC);
                force_visible = true;
                break;
            case 0:     //STABILIZE =     0,  // manual airframe angle with manual throttle
            case 1:     //ACRO =          1,  // manual body-frame angular rate with manual throttle
            case 2:     //ALT_HOLD =      2,  // manual airframe angle with automatic throttle
            case 3:     //AUTO =          3,  // fully automatic waypoint control using mission commands
            case 4:     //GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
            case 5:     //LOITER =        5,  // automatic horizontal acceleration with automatic throttle
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
    // do we need to show the DJI flight mode?
    if (force_visible) {
        BIT_CLEAR(osd_hidden_items_bitmask, OSD_FLYMODE);
    }
    return mode_mask;
}

bool AP_MSP_Telem_DJI::msp_process_out_status(sbuf_t *dst)
{
    // DJI OSD relies on a statically defined bit order and doesn't use MSP_BOXIDS
    // to get actual BOX order. We need a special packBoxModeFlags()
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

bool AP_MSP_Telem_DJI::msp_process_out_api_version(sbuf_t *dst)
{
    sbuf_write_u8(dst, MSP_PROTOCOL_VERSION);
    sbuf_write_u8(dst, API_VERSION_MAJOR);
    sbuf_write_u8(dst, API_VERSION_MINOR);
    return true;
}

bool AP_MSP_Telem_DJI::msp_process_out_fc_version(sbuf_t *dst)
{
   sbuf_write_u8(dst, FC_VERSION_MAJOR);
   sbuf_write_u8(dst, FC_VERSION_MINOR);
   sbuf_write_u8(dst, FC_VERSION_PATCH_LEVEL);
   return true;
}

bool AP_MSP_Telem_DJI::msp_process_out_fc_variant(sbuf_t *dst)
{
   sbuf_write_data(dst, "BTFL", FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
   return true;
}

#endif  //HAL_MSP_ENABLED