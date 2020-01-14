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

static const uint8_t AUTOPILOT_NODE_ID = 0;
static const uint8_t WING_ANGLE_SERVO_NODE_ID = 0x96;

static const uint8_t GET_PWM_OBJ_ADDR = 1;
static const uint8_t SET_PWM_OBJ_ADDR = 2;

static const uint8_t SEND_PWM_PERIOD_MS = 200;

static const uint8_t MAX_XMIT_RETRIES = 5;

class AP_WingAngleServo : public AP_TinCANClient
{
public:
    AP_WingAngleServo();
    ~AP_WingAngleServo() {};

    /* Do not allow copies */
    AP_WingAngleServo(const AP_WingAngleServo &other) = delete;
    AP_WingAngleServo &operator=(const AP_WingAngleServo &) = delete;

    void init();

    bool receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame) override;
    bool transmit_slot(uint8_t interface_index) override;

    void set_output_pwm(uint16_t _output_pwm)
    {
        output_pwm = _output_pwm;
    }

private:
    uint16_t    output_pwm = 0;
    uint32_t    next_pwm_val_send_ms = 0;
    AP_TinCAN * p_tincan = nullptr;
    uint8_t     n_xmit_errors_in_burst = 0;

};
#endif
