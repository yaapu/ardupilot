#include "GCS_Mavlink.h"

uint32_t GCS_MAVLINK_Frsky::telem_delay() const  { return 0; }

void GCS_MAVLINK_Frsky::handleMessage(const mavlink_message_t &msg)  {}

bool GCS_MAVLINK_Frsky::try_send_message(enum ap_message id)  { return true; }

bool GCS_MAVLINK_Frsky::handle_guided_request(AP_Mission::Mission_Command &cmd)  { return true; }

void GCS_MAVLINK_Frsky::handle_change_alt_request(AP_Mission::Mission_Command &cmd)  {}

uint8_t GCS_MAVLINK_Frsky::sysid_my_gcs() const  { return 1; }

MAV_MODE GCS_MAVLINK_Frsky::base_mode() const  { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }

MAV_STATE GCS_MAVLINK_Frsky::vehicle_system_status() const  { return MAV_STATE_CALIBRATING; }

bool GCS_MAVLINK_Frsky::set_home_to_current_location(bool _lock)  { return false; }

bool GCS_MAVLINK_Frsky::set_home(const Location& loc, bool _lock)  { return false; }

void GCS_MAVLINK_Frsky::send_nav_controller_output() const  {}

void GCS_MAVLINK_Frsky::send_pid_tuning()  {}