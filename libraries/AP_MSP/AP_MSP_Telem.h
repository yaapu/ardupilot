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

#include "AP_MSP_Telem_Backend.h"

#if  HAL_MSP_ENABLED

class AP_MSP_Telem : public AP_MSP_Telem_Backend
{
public:
    AP_MSP_Telem(AP_MSP& msp, uint8_t protocol_instance, bool scheduler_enabled);

    ~AP_MSP_Telem();

    uint32_t get_osd_flight_mode_bitmask(void) override;
    bool msp_process_out_api_version(sbuf_t *dst) override;
    bool msp_process_out_fc_version(sbuf_t *dst) override;
    bool msp_process_out_fc_variant(sbuf_t *dst) override;
};

#endif  //HAL_MSP_ENABLED