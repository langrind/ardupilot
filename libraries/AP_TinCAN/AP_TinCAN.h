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

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#define TINCAN_MAX_CLIENTS 3

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

class AP_TinCANClient {
public:
    /* give the client an opportunity to process a received frame */
    virtual bool receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame) { return false; }
    
    /* give the client a transmit slot */
    virtual bool transmit_slot(uint8_t interface_index, uavcan::CanFrame &recv_frame) { return false; }

    /* Does the client want to receive everything? */
    virtual bool is_greedy(uint8_t interface_index) { return false; }

    /* Then the two problematic interfaces: esc telemetry and servo output, punting for now */

    /* Then the question of how to coordinate transmit slots */

    int get_transmit_period() { return transmit_period_ms; }
    int get_transmit_offset() { return transmit_offset_ms; }
    void set_transmit_offset(int _transmit_offset_ms ) { transmit_offset_ms = _transmit_offset_ms; }

protected:
    int transmit_period_ms;
    int transmit_offset_ms;

};

class AP_TinCAN : public AP_HAL::CANProtocol {
public:
    AP_TinCAN();
    ~AP_TinCAN();

    /* Do not allow copies */
    AP_TinCAN(const AP_TinCAN &other) = delete;
    AP_TinCAN &operator=(const AP_TinCAN&) = delete;

    // Return TinCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_TinCAN *get_tcan(uint8_t driver_index);

    // initialise TinCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // write frame on CAN bus, returns true on success
    bool write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout);

    // client calls this to register with us
    int add_client(AP_TinCANClient * client);
    
private:

    // loop to perform all activity
    void loop();

    void dispatch_frame( uint8_t interface_index, uavcan::CanFrame &recv_frame );

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;
    const uavcan::CanFrame* _select_frames[uavcan::MaxCanIfaces] { };

    AP_TinCANClient * client_array[TINCAN_MAX_CLIENTS];
    
};

