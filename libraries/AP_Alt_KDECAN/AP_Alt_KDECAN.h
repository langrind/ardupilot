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
 * AP_Alt_KDECAN.h
 *
 *      Author: Francisco Ferreira
 *
 * Hacked up by Nik Langrind to experiment with different CAN Mux layer
 * This alternative implementation only supports telemetry, no
 * dynamic enumeration, no throttle command
 */

#pragma once

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#include <AP_Param/AP_Param.h>

#include <atomic>

#include <AP_TinCAN/AP_TinCAN.h>

/* max supported in our application is 2, real KDECAN supports more */
#define KDECAN_MAX_NUM_ESCS 2

class AP_Alt_KDECAN : public AP_TinCANClient
{
public:
    AP_Alt_KDECAN();

    /* Do not allow copies */
    AP_Alt_KDECAN(const AP_Alt_KDECAN &other) = delete;
    AP_Alt_KDECAN &operator=(const AP_Alt_KDECAN&) = delete;

    void init();

    // send MAVLink telemetry packets
    void send_mavlink(uint8_t chan);

    bool receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame) override;
    bool transmit_slot(uint8_t interface_index) override;


private:
    bool                _initialized;
    char                _thread_name[11];
    uint8_t             _driver_index;
    uavcan::ICanDriver* _can_driver;
    uint32_t            next_send_ms;
    uint8_t             next_send_destination_id;
    uint32_t            n_xmit_errors;

    // telemetry input
    HAL_Semaphore _telem_sem;
    struct telemetry_info_t {
        uint64_t time;
        uint16_t voltage;
        uint16_t current;
        uint16_t rpm;
        uint8_t temp;
        bool new_data;
    } _telemetry[KDECAN_MAX_NUM_ESCS];
    AP_TinCAN * p_tincan = nullptr;

#if 0
    union frame_id_t {
        struct {
            uint8_t object_address;
            uint8_t destination_id;
            uint8_t source_id;
            uint8_t priority:5;
            uint8_t unused:3;
        };
        uint32_t value;
    };
#endif
    static const uint8_t AUTOPILOT_NODE_ID = 0;
    static const uint8_t BROADCAST_NODE_ID = 1;
    static const uint8_t ESC_NODE_ID_FIRST = 2;

    static const uint8_t ESC_INFO_OBJ_ADDR = 0;
    static const uint8_t SET_PWM_OBJ_ADDR = 1;
    static const uint8_t VOLTAGE_OBJ_ADDR = 2;
    static const uint8_t CURRENT_OBJ_ADDR = 3;
    static const uint8_t RPM_OBJ_ADDR = 4;
    static const uint8_t TEMPERATURE_OBJ_ADDR = 5;
    static const uint8_t GET_PWM_INPUT_OBJ_ADDR = 6;
    static const uint8_t GET_PWM_OUTPUT_OBJ_ADDR = 7;
    static const uint8_t MCU_ID_OBJ_ADDR = 8;
    static const uint8_t UPDATE_NODE_ID_OBJ_ADDR = 9;
    static const uint8_t ENUM_OBJ_ADDR = 10;
    static const uint8_t TELEMETRY_OBJ_ADDR = 11;

    static const uint16_t SET_PWM_MIN_INTERVAL_US = 2500;
    static const uint32_t TELEMETRY_INTERVAL_US = 100000;

    static const uint32_t SET_PWM_TIMEOUT_US = 2000;
    static const uint16_t TELEMETRY_TIMEOUT_US = 500;
    static const uint16_t ENUMERATION_TIMEOUT_MS = 30000;
    static const uint8_t  TELEMETRY_REQUEST_PERIOD_MS = 50;

    static const uint8_t CAN_IFACE_INDEX = 0;
};
