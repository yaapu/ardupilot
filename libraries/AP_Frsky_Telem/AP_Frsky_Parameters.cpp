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
#include "AP_Frsky_Parameters.h"

const AP_Param::GroupInfo AP_Frsky_Parameters::var_info[] = {
    // @Param: UPLINK SENSOR ID
    // @DisplayName: Select frsky passthrough telemetry uplink sensor id
    // @Description: Change the default uplink sensor id, requires reboot
    // @Values: 0=Disable, 8 - 27
    // @User: Advanced
    AP_GROUPINFO("UPLINK_ID",  1, AP_Frsky_Parameters, _sport_uplink_id, FRSKY_UPLINK_ID_DEFAULT),

    // @Param: DOWNLINK SENSOR ID
    // @DisplayName: Select frsky passthrough telemetry additional first downlink sensor id
    // @Description: Change the default extra downlink sensor id, requires reboot
    // @Values: 0=Disable, 8 - 27
    // @User: Advanced
    AP_GROUPINFO("DNLINK1_ID",  2, AP_Frsky_Parameters, _sport_dnlink1_id, FRSKY_DNLINK_ID1_DEFAULT),

    // @Param: UPLINK SENSOR ID
    // @DisplayName: Select frsky passthrough telemetry additional second downlink sensor id
    // @Description: Change the default extra downlink sensor id, requires reboot
    // @Values: 0=Disable, 8 - 27
    // @User: Advanced
    AP_GROUPINFO("DNLINK2_ID",  3, AP_Frsky_Parameters, _sport_dnlink2_id, FRSKY_DNLINK_ID2_DEFAULT),

    AP_GROUPEND
};

AP_Frsky_Parameters::AP_Frsky_Parameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}