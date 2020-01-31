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
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef TINCAN_ENABLED
#include <AP_HAL/CAN.h>
#include <AP_TinCAN/AP_TinCAN.h>

class AP_WingAngleSensor : public AP_TinCANClient
{
public:
    AP_WingAngleSensor();
    ~AP_WingAngleSensor() {};

    /* Do not allow copies */
    AP_WingAngleSensor(const AP_WingAngleSensor &other) = delete;
    AP_WingAngleSensor &operator=(const AP_WingAngleSensor &) = delete;

    // get singleton instance
    static AP_WingAngleSensor *get_singleton(void) {
        return _singleton;
    }

    void init();

    bool receive_frame(uint8_t interface_index, const uavcan::CanFrame &recv_frame) override;

    uint16_t get_left_sensor_val() { return _left_sensor_val; }
    uint16_t get_right_sensor_val() { return _right_sensor_val; }

private:
    uint16_t _left_sensor_val;
    uint16_t _right_sensor_val;
    uint64_t _last_sensor_update_us;

    static AP_WingAngleSensor *_singleton;
};
#endif

