#include "AP_Airspeed_MSP.h"

#if AP_AIRSPEED_MSP_ENABLED

extern const AP_HAL::HAL& hal;

AP_Airspeed_MSP::AP_Airspeed_MSP(AP_Airspeed &_frontend, uint8_t _instance, uint8_t _msp_instance) :
    AP_Airspeed_Backend(_frontend, _instance),
    msp_instance(_msp_instance)
{
    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MSP,0,msp_instance,0));
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_MSP::get_differential_pressure(float &pressure)
{
    hal.console->printf("MSP: count_pres: %i, sum_press:%.02f\n", press_count, sum_pressure);
    WITH_SEMAPHORE(sem);
    if (press_count == 0) {
        return false;
    }
    pressure = sum_pressure/press_count;
    press_count = 0;
    sum_pressure = 0;
    return true;
}

// get last temperature
bool AP_Airspeed_MSP::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);
    if (temp_count == 0) {
        return false;
    }
    temperature = sum_temp/temp_count;
    temp_count = 0;
    sum_temp = 0;
    return true;
}

void AP_Airspeed_MSP::handle_msp(const MSP::msp_airspeed_data_message_t &pkt)
{
    if (pkt.instance != msp_instance) {
        // not for us
        return;
    }
    WITH_SEMAPHORE(sem);

    sum_pressure += pkt.pressure;
    press_count++;
    if (press_count > 100) {
        // prevent overflow
        sum_pressure /= 2;
        press_count /= 2;
    }
    hal.console->printf("MSP: tm=%lu, inst:%i, p=%0.2f, t=%0.2f\n", pkt.time_ms, pkt.instance, pkt.pressure, pkt.pressure);

    if (pkt.temp == INT16_MIN) {
        // invalid temperature
        return;
    }

    sum_temp += pkt.temp*0.01;
    temp_count++;
    if (temp_count > 100) {
        // prevent overflow
        sum_temp /= 2;
        temp_count /= 2;
    }

}

#endif // AP_AIRSPEED_MSP_ENABLED
