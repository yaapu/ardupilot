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
    The DJI Air Unit polls the FC for the following MSP messages at around 4Hz.
    Note: messages are polled in ascending hex id order.
    
    Hex|Dec|Name
    ---------------------------
    03	03	MSP_FC_VERSION
    0a	10	MSP_NAME
    54	84	MSP_OSD_CONFIG
    5c	92	MSP_FILTER_CONFIG
    5e	94	MSP_PID_ADVANCED
    65	101	MSP_STATUS
    69	105	MSP_RC
    6a	106	MSP_RAW_GPS
    6b	107	MSP_COMP_GPS
    6c	108	MSP_ATTITUDE
    6d	109	MSP_ALTITUDE
    6e	110	MSP_ANALOG
    6f	111	MSP_RC_TUNING
    70	112	MSP_PID
    82	130	MSP_BATTERY_STATE
    86	134	MSP_ESC_SENSOR_DATA
    96	150	MSP_STATUS_EX
    f7	247	MSP_RTC    
*/
#include <AP_Common/AP_FWVersion.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

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
    if (_notify.flags.failsafe_battery || _notify.flags.failsafe_gcs || _notify.flags.failsafe_radio || _notify.flags.ekf_bad ) {
        BIT_SET(mode_mask, DJI_MODE_FS);
        force_visible = true;
    }

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    switch (AP::notify().flags.flight_mode) {
        case 11:    // RTL
        case 21:    // QRTL
            BIT_SET(mode_mask, DJI_MODE_RESC);
            force_visible = true;
            break;
        default:
            break;
    }
#endif
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    switch (AP::notify().flags.flight_mode) {
        case 6:     // RTL
        case 21:    // SMART RTL
            BIT_SET(mode_mask, DJI_MODE_RESC);
            force_visible = true;
            break;
        default:
            break;
    }
#endif
#if APM_BUILD_TYPE(APM_BUILD_Rover)
    switch (AP::notify().flags.flight_mode) {
        case 11:     // RTL
        case 12:     // SMART RTL
            BIT_SET(mode_mask, DJI_MODE_RESC);
            force_visible = true;
            break;
        default:
            break;
    }
#endif
    // do we need to show the DJI flight mode?
    if (force_visible) {
        BIT_CLEAR(osd_hidden_items_bitmask, OSD_FLYMODE);
    }
    return mode_mask;
}

bool AP_MSP_Telem_DJI::msp_process_out_status(sbuf_t *dst)
{
    // DJI OSD relies on a statically defined bit order
    // We need a special get_osd_flight_mode_bitmask()
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