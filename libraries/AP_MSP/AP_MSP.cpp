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
   MSP protocol library
*/

#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RSSI/AP_RSSI.h>
#include <ctype.h>
#include <stdio.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem.h"
#include "AP_MSP_Telem_DJI.h"

#if HAL_MSP_ENABLED


#define OSD_FLIGH_MODE_FOCUS_TIME 2000
//#define DEBUG_MSP_CMD
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MSP::var_info[] = {

    // @Param: _OSD_UNITS
    // @DisplayName: Display Units
    // @Description: Sets the units to use in displaying items
    // @Values: 0:Imperial,1:Metric
    // @User: Standard
    AP_GROUPINFO("_OSD_UNITS", 1, AP_MSP, units, 0),

    // @Param: _OSD_MSGTIME
    // @DisplayName: Message display duration in seconds
    // @Description: Sets message duration seconds
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("_OSD_MSGTIME", 2, AP_MSP, msgtime_s, 10),

    // @Param: _OSD_NCELLS
    // @DisplayName: Force cell count
    // @Description: Force cell count
    // @Values: 0:Auto,1 -12
    // @User: Standard
    AP_GROUPINFO("_OSD_NCELLS", 3, AP_MSP, cellcount, 0),

    // @Param: _OSD_WIND_EN
    // @DisplayName: Enable wind
    // @Description: Enable wind
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_OSD_WIND_EN", 4, AP_MSP, wind_en, 0),

    // @Param: _OSD_ASPD_EN
    // @DisplayName: Enable airspeed override
    // @Description: Enable airspeed override
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_OSD_ASPD_EN", 5, AP_MSP, airspeed_en, 0),

    // @Param: _OSD_RSSI_EN
    // @DisplayName: _OSD_RSSI_EN
    // @Description: Displays RC signal strength
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_RSSI_X
    // @DisplayName: _OSD_RSSI_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_RSSI_Y
    // @DisplayName: _OSD_RSSI_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rssi, "_OSD_RSSI", 6, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_ALT_EN
    // @DisplayName: _OSD_ALT_EN
    // @Description: Enables display of altitude AGL
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_ALT_X
    // @DisplayName: _OSD_ALT_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_ALT_Y
    // @DisplayName: _OSD_ALT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(alt, "_OSD_ALT", 7, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_VBAT_EN
    // @DisplayName: _OSD_VBAT_EN
    // @Description: Displays main battery voltage
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_VBAT_X
    // @DisplayName: _OSD_VBAT_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_VBAT_Y
    // @DisplayName: _OSD_VBAT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vbat, "_OSD_VBAT", 8, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_CURR_EN
    // @DisplayName: _OSD_CURR_EN
    // @Description: Displays main battery current
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_CURR_X
    // @DisplayName: _OSD_CURR_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_CURR_Y
    // @DisplayName: _OSD_CURR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(curr, "_OSD_CURR", 9, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_MAH_Y_EN
    // @DisplayName: _OSD_MAH_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_MAH_Y_X
    // @DisplayName: _OSD_MAH_Y_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_MAH_Y
    // @DisplayName: _OSD_MAH_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(mah, "_OSD_MAH", 10, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_SATS_EN
    // @DisplayName: _OSD_SATS_EN
    // @Description: Displays number of acquired sattelites
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_SATS_X
    // @DisplayName: _OSD_SATS_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_SATS_Y
    // @DisplayName: _OSD_SATS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(nsats, "_OSD_SATS", 11, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_FMODE_EN
    // @DisplayName: _OSD_FMODE_EN
    // @Description: Displays flight mode
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_FMODE_X
    // @DisplayName: _OSD_FMODE_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_FMODE_Y
    // @DisplayName: _OSD_FMODE_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(flmode, "_OSD_FMODE", 12, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_MSG_EN
    // @DisplayName: _OSD_MSG_EN
    // @Description: Displays Mavlink messages
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_MSG_X
    // @DisplayName: _OSD_MSG_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_MSG_Y
    // @DisplayName: _OSD_MSG_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(name, "_OSD_MSG", 13, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_GSPD_EN
    // @DisplayName: _OSD_GSPD_EN
    // @Description: Displays GPS ground speed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_GSPD_X
    // @DisplayName: _OSD_GSPD_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_GSPD_Y
    // @DisplayName: _OSD_GSPD_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(gspd, "_OSD_GSPD", 14, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_HORIZ_EN
    // @DisplayName: _OSD_HORIZ_EN
    // @Description: Displays artificial horizon
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_HORIZ_X
    // @DisplayName: _OSD_HORIZ_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_HORIZ_Y
    // @DisplayName: _OSD_HORIZ_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(ahor, "_OSD_HORIZ", 15, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_HDIST_EN
    // @DisplayName: _OSD_HDIST_EN
    // @Description: Displays distance and relative direction to HOME
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_HDIST_X
    // @DisplayName: _OSD_HDIST_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_HDIST_Y
    // @DisplayName: _OSD_HDIST_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hdist, "_OSD_HDIST", 16, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_HDIR_EN
    // @DisplayName: _OSD_HDIR_EN
    // @Description: Displays distance and relative direction to HOME
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_HDIR_X
    // @DisplayName: _OSD_HDIR_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_HDIR_Y
    // @DisplayName: _OSD_HDIR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(hdir, "_OSD_HDIR", 17, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_LAT_EN
    // @DisplayName: _OSD_LAT_EN
    // @Description: Displays GPS latitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_LAT_X
    // @DisplayName: _OSD_LAT_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_LAT_Y
    // @DisplayName: _OSD_LAT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(lat, "_OSD_LAT", 18, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_LON_EN
    // @DisplayName: _OSD_LON_EN
    // @Description: Displays GPS longitude
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_LON_X
    // @DisplayName: _OSD_LON_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_LON_Y
    // @DisplayName: _OSD_LON_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(lon, "_OSD_LON", 19, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_ROLL_EN
    // @DisplayName: _OSD_ROLL_EN
    // @Description: Displays degrees of roll from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_ROLL_X
    // @DisplayName: _OSD_ROLL_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_ROLL_Y
    // @DisplayName: _OSD_ROLL_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(roll, "_OSD_ROLL", 20, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_PITCH_EN
    // @DisplayName: _OSD_PITCH_EN
    // @Description: Displays degrees of pitch from level
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_PITCH_X
    // @DisplayName: _OSD_PITCH_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_PITCH_Y
    // @DisplayName: _OSD_PITCH_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(pitch, "_OSD_PITCH", 21, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_BATT_EN
    // @DisplayName: _OSD_BATT_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_BATT_X
    // @DisplayName: _OSD_BATT_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_BATT_Y
    // @DisplayName: _OSD_BATT_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(battusg, "_OSD_BATT", 22, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_VSPD_EN
    // @DisplayName: _OSD_VSPD_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_VSPD_X
    // @DisplayName: _OSD_VSPD_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_VSPD_Y
    // @DisplayName: _OSD_VSPD_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(vario, "_OSD_VSPD", 23, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_ARM_EN
    // @DisplayName: _OSD_ARM_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_ARM_X
    // @DisplayName: _OSD_ARM_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_ARM_Y
    // @DisplayName: _OSD_ARM_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(arm, "_OSD_ARM", 24, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_ACELL_EN
    // @DisplayName: _OSD_ACELL_EN
    // @Description: Displays primary battery mAh consumed
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_ACELL_X
    // @DisplayName: _OSD_ACELL_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_ACELL_Y
    // @DisplayName: _OSD_ACELL_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cell, "_OSD_VCELL", 25, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_PWR_EN
    // @DisplayName: _OSD_PWR_EN
    // @Description: Displays power
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_PWR_X
    // @DisplayName: _OSD_PWR_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_PWR_Y
    // @DisplayName: _OSD_PWR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(power, "_OSD_PWR", 26, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_RTC_EN
    // @DisplayName: _OSD_RTC_EN
    // @Description: Displays RTC
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_RTC_X
    // @DisplayName: _OSD_RTC_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_RTC_Y
    // @DisplayName: _OSD_RTC_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(rtc, "_OSD_RTC", 27, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_TESC_EN
    // @DisplayName: _OSD_TESC_EN
    // @Description: Displays RTC
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_TESC_X
    // @DisplayName: _OSD_TESC_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_TESC_Y
    // @DisplayName: _OSD_TESC_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(esctemp, "_OSD_TESC", 28, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_CROSS_EN
    // @DisplayName: _OSD_HORIZ_EN
    // @Description: Displays crosshair
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_CROSS_X
    // @DisplayName: _OSD_CROSS_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_CROSS_Y
    // @DisplayName: _OSD_CROSS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(cross, "_OSD_CROSS", 29, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_HBARS_EN
    // @DisplayName: _OSD_HBARS_EN
    // @Description: Displays artificial horizon ladders
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_HBARS_X
    // @DisplayName: _OSD_HBARS_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_HBARS_Y
    // @DisplayName: _OSD_HBARS_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(ahbars, "_OSD_HBARS", 30, AP_MSP, AP_OSD_Setting),

    // @Param: _OSD_THR_EN
    // @DisplayName: _OSD_THR_EN
    // @Description: Displays throttle position
    // @Values: 0:Disabled,1:Enabled

    // @Param: _OSD_THR_X
    // @DisplayName: _OSD_THR_X
    // @Description: Horizontal position on screen
    // @Range: 0 27

    // @Param: _OSD_THR_Y
    // @DisplayName: _OSD_THR_Y
    // @Description: Vertical position on screen
    // @Range: 0 15
    AP_SUBGROUPINFO(throttle, "_OSD_THR", 31, AP_MSP, AP_OSD_Setting),

    AP_GROUPEND
};

AP_MSP *AP_MSP::singleton;

AP_MSP::AP_MSP()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

AP_MSP::~AP_MSP(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
bool AP_MSP::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    uint8_t msp_idx[BACKEND_COUNT] {};
    uint8_t backend_count = 0;
    for (uint8_t i=0; i<MSP_MAX_INSTANCES; i++) {
        if ((msp_port[i].uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DJI_FPV, msp_idx[BACKEND_DJI]))) {
            msp_port[i].protocol = AP_SerialManager::SerialProtocol_DJI_FPV;
            msp_port[i].telem_backend = new AP_MSP_Telem_DJI(*this, i, true);
            if (msp_port[i].telem_backend != nullptr) {
                msp_port[i].telem_backend->init();
            }
            msp_idx[BACKEND_DJI]++;
            backend_count++;
        } else if ((msp_port[i].uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MSP, msp_idx[BACKEND_MSP]))) {
            msp_port[i].protocol = AP_SerialManager::SerialProtocol_MSP;
            msp_port[i].telem_backend = new AP_MSP_Telem(*this, i, false);
            if (msp_port[i].telem_backend != nullptr) {
                msp_port[i].telem_backend->init();
            }
            msp_idx[BACKEND_MSP]++;
            backend_count++;
        }
    }

    if (backend_count > 0) {
        // we've found at least 1 msp decoder, start protocol handler
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSP::loop, void),
                                          "MSP",
                                          1024, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
            return false;
        }
    }
    return false;
}

void AP_MSP::loop(void)
{
    for (uint8_t i=0; i<MSP_MAX_INSTANCES; i++) {
        // one time uart init
        if (msp_port[i].uart != nullptr) {
            msp_port[i].uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            msp_port[i].uart->begin(0, AP_SERIALMANAGER_MSP_BUFSIZE_RX, AP_SERIALMANAGER_MSP_BUFSIZE_TX);
        }
        // one time osd item overrides (overrides require reboot)
        if (msp_port[i].telem_backend != nullptr) {
            msp_port[i].telem_backend->apply_osd_items_overrides();
        }
    }

    bool update_flashing;
    while (true) {
        hal.scheduler->delay(1);

        // flashing timer
        uint32_t now = AP_HAL::millis();
        update_flashing = false;
        // toggle flashing every 0.7 seconds
        if ((now - flash_start_time) > 700) {
            flashing_on = !flashing_on;
            flash_start_time = now;
            update_flashing = true;
        }

        // detect flight mode changes and steal focus from text messages
        if (AP::notify().flags.flight_mode != last_flight_mode) {
            flight_mode_focus = true;
            last_flight_mode = AP::notify().flags.flight_mode;
            last_flight_mode_change = AP_HAL::millis();
        }

        if (now - last_flight_mode_change > OSD_FLIGH_MODE_FOCUS_TIME) {
            flight_mode_focus = false;
        }

        for (uint8_t i=0; i< MSP_MAX_INSTANCES; i++) {
            if (msp_port[i].telem_backend != nullptr) {
                // dynamically hide/unhide only if flashing status changed
                if (update_flashing) {
                    //gcs().send_text(MAV_SEVERITY_DEBUG, "pos:%d, mode:%d", MSP_OSD_POS(osd_item_settings[OSD_FLYMODE]),msp_port[i].telem_backend->get_osd_flight_mode_bitmask());
                    msp_port[i].telem_backend->flash_osd_items();
                }
                // process incoming MSP frames
                process_serial_data(i);
                // push telemetry frames
                msp_port[i].telem_backend->run_wfq_scheduler();
            }
        }
    }
}

/*
    invoked by the WFQ telemetry scheduler as a callback at each wfq step
*/
void AP_MSP::msp_telemetry_callback(uint16_t cmd_msp, uint8_t protocol_instance)
{
#ifdef DEBUG_MSP_CMD
    gcs().send_text(MAV_SEVERITY_DEBUG,"cmd_msp=%d, instance=%d",cmd_msp, protocol_instance);
#endif
    uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    msp_packet_t reply = {
        .buf = { .ptr = outBuf, .end = MSP_ARRAYEND(outBuf), },
        .cmd = (int16_t)cmd_msp,
        .flags = 0,
        .result = 0,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    msp_process_out_command(&msp_port[protocol_instance], cmd_msp, &reply.buf);
    sbuf_switch_to_reader(&reply.buf, outBufHead); // change streambuf direction
    msp_serial_encode(&msp_port[protocol_instance], &reply, msp_port[protocol_instance].msp_version);

    msp_port[protocol_instance].c_state = MSP_IDLE;

}

void AP_MSP::msp_handle_opflow(const msp_opflow_sensor_t &pkt)
{
    OpticalFlow *optflow = AP::opticalflow();
    if (optflow == nullptr) {
        return;
    }
    optflow->handle_msp(pkt);
}

void AP_MSP::msp_handle_rangefinder(const msp_rangefinder_sensor_t &pkt)
{
    RangeFinder *rangefinder = AP::rangefinder();
    if (rangefinder == nullptr) {
        return;
    }
    rangefinder->handle_msp(pkt);
}

/*
 * read serial
 */
void AP_MSP::process_serial_data(uint8_t instance)
{
    if (msp_port[instance].uart == nullptr) {
        return;
    }

    uint32_t numc = msp_port[instance].uart->available();

    if (numc > 0) {
        // There are bytes incoming
        msp_port[instance].last_activity_ms = AP_HAL::millis();

        // Process incoming bytes
        while (numc-- > 0) {
            const uint8_t c = msp_port[instance].uart->read();
            msp_parse_received_data(&msp_port[instance], c);

            if (msp_port[instance].c_state == MSP_COMMAND_RECEIVED) {
                msp_process_received_command(&msp_port[instance]);
            }
        }
    }
}

void AP_MSP::msp_process_received_command(msp_port_t *msp)
{
    uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    msp_packet_t reply = {
        .buf = { .ptr = outBuf, .end = MSP_ARRAYEND(outBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    msp_packet_t command = {
        .buf = { .ptr = msp->in_buf, .end = msp->in_buf + msp->data_size, },
        .cmd = (int16_t)msp->cmd_msp, // this cast looks suspicious but it's the same in inav, compiler checks in inav are less strict?
        .flags = msp->cmd_flags,
        .result = 0,
    };

    const msp_result_e status = msp_process_command(msp, &command, &reply);

    if (status != MSP_RESULT_NO_REPLY) {
        sbuf_switch_to_reader(&reply.buf, outBufHead); // change streambuf direction
        msp_serial_encode(msp, &reply, msp->msp_version);
    }

    msp->c_state = MSP_IDLE;
}

msp_result_e AP_MSP::msp_process_command(msp_port_t *msp, msp_packet_t *cmd, msp_packet_t *reply)
{
#ifdef DEBUG_MSP_CMD
    gcs().send_text(MAV_SEVERITY_DEBUG,"cmd_msp=%d", cmd_msp);
#endif
    msp_result_e ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (MSP2_IS_SENSOR_MESSAGE(cmdMSP)) {
        ret = msp_process_sensor_command(msp, cmdMSP, src);
    } else if (msp_process_out_command(msp, cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    }

    // Process DONT_REPLY flag
    if (cmd->flags & MSP_FLAG_DONT_REPLY) {
        ret = MSP_RESULT_NO_REPLY;
    }

    reply->result = ret;
    return ret;
}

bool AP_MSP::msp_process_out_command(msp_port_t *msp, uint16_t cmd_msp, sbuf_t *dst)
{
    if (msp->telem_backend == nullptr) {
        return false;
    }

    switch (cmd_msp) {
        case MSP_API_VERSION:
            return msp->telem_backend->msp_process_out_api_version(dst);
        case MSP_FC_VARIANT:
            return msp->telem_backend->msp_process_out_fc_variant(dst);
        case MSP_FC_VERSION:
            return msp->telem_backend->msp_process_out_fc_version(dst);
        case MSP_BOARD_INFO:
            return msp->telem_backend->msp_process_out_board_info(dst);
        case MSP_BUILD_INFO:
            return msp->telem_backend->msp_process_out_build_info(dst);
        case MSP_NAME:
            return msp->telem_backend->msp_process_out_name(dst);
        case MSP_OSD_CONFIG:
            return msp->telem_backend->msp_process_out_osd_config(dst);
        case MSP_STATUS:
        case MSP_STATUS_EX:
            return msp->telem_backend->msp_process_out_status(dst);
        case MSP_RAW_GPS:
            return msp->telem_backend->msp_process_out_raw_gps(dst);
        case MSP_COMP_GPS:
            return msp->telem_backend->msp_process_out_comp_gps(dst);
        case MSP_ATTITUDE:
            return msp->telem_backend->msp_process_out_attitude(dst);
        case MSP_ALTITUDE:
            return msp->telem_backend->msp_process_out_altitude(dst);
        case MSP_ANALOG:
            return msp->telem_backend->msp_process_out_analog(dst);
        case MSP_BATTERY_STATE:
            return msp->telem_backend->msp_process_out_battery_state(dst);
        case MSP_UID:
            return msp->telem_backend->msp_process_out_uid(dst);
#ifdef HAVE_AP_BLHELI_SUPPORT
        case MSP_ESC_SENSOR_DATA:
            return msp->telem_backend->msp_process_out_esc_sensor_data(dst);
#endif
        case MSP_RTC:
            return msp->telem_backend->msp_process_out_rtc(dst);
        case MSP_RC:
            return msp->telem_backend->msp_process_out_rc(dst);
        default:
            return false;
    }
}

msp_result_e AP_MSP::msp_process_sensor_command(msp_port_t* msp, uint16_t cmd_msp, sbuf_t *src)
{
    MSP_UNUSED(src);

    switch (cmd_msp) {
        case MSP2_SENSOR_RANGEFINDER: {
            const msp_rangefinder_sensor_t pkt = *(const msp_rangefinder_sensor_t *)src->ptr;
            msp_handle_rangefinder(pkt);
        }
        break;
        case MSP2_SENSOR_OPTIC_FLOW: {
            const msp_opflow_sensor_t pkt = *(const msp_opflow_sensor_t *)src->ptr;
            msp_handle_opflow(pkt);
        }
        break;
    }

    return MSP_RESULT_NO_REPLY;
}

namespace AP
{
AP_MSP *msp()
{
    return AP_MSP::get_singleton();
}
};

#endif  //HAL_MSP_ENABLED