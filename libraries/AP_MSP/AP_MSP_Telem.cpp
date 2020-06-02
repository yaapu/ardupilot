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
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem.h"

#if  HAL_MSP_ENABLED

extern const AP_HAL::HAL& hal;


AP_MSP_Telem::AP_MSP_Telem(AP_MSP& msp, uint8_t protocol_instance, bool scheduler_enabled) : AP_MSP_Telem_Backend(msp, protocol_instance, scheduler_enabled)
{
}

AP_MSP_Telem::~AP_MSP_Telem(void)
{
}

bool AP_MSP_Telem::msp_process_out_api_version(sbuf_t *dst)
{
    sbuf_write_u8(dst, MSP_PROTOCOL_VERSION);
    sbuf_write_u8(dst, API_VERSION_MAJOR);
    sbuf_write_u8(dst, API_VERSION_MINOR);
    return true;
}

bool AP_MSP_Telem::msp_process_out_fc_version(sbuf_t *dst)
{
    sbuf_write_u8(dst, FC_VERSION_MAJOR);
    sbuf_write_u8(dst, FC_VERSION_MINOR);
    sbuf_write_u8(dst, FC_VERSION_PATCH_LEVEL);
    return true;
}

bool AP_MSP_Telem::msp_process_out_fc_variant(sbuf_t *dst)
{
    sbuf_write_data(dst, "ARDU", FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    return true;
}

uint32_t AP_MSP_Telem::get_osd_flight_mode_bitmask(void)
{
    BIT_SET(osd_hidden_items_bitmask, OSD_FLYMODE);
    return 0;
}

#endif  //HAL_MSP_ENABLED