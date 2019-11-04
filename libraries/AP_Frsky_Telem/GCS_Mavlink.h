#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Frsky : public GCS_MAVLINK
{
public:
    friend class AP_Frsky_Telem; //to access protected command and camera handlers
    
    using GCS_MAVLINK::GCS_MAVLINK;

private:

    uint32_t telem_delay() const override;
    void handleMessage(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;

protected:

    uint8_t sysid_my_gcs() const override;

    // dummy information:
    MAV_MODE base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    bool set_home_to_current_location(bool _lock) override;
    bool set_home(const Location& loc, bool _lock) override;

    void send_nav_controller_output() const override;
    void send_pid_tuning() override;
};