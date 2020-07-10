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

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 1
#endif

/* Parameters defaults */
#define FRSKY_UPLINK_ID_DEFAULT     13
#define FRSKY_DNLINK_ID1_DEFAULT    20
#define FRSKY_DNLINK_ID2_DEFAULT    7

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

class AP_Frsky_Telem;

class AP_Frsky_Parameters
{
public:
    friend AP_Frsky_Telem;
    AP_Frsky_Parameters();

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // settable parameters
    AP_Int8 _sport_uplink_id;
    AP_Int8 _sport_dnlink1_id;
    AP_Int8 _sport_dnlink2_id;
};

#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL