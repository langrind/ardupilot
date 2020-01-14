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

    void init();

    bool receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame) override;

private:
    uint16_t left_sensor_val;
    uint16_t right_sensor_val;
    uint64_t last_sensor_update_us;
};
#endif

