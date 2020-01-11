
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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef TINCAN_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_TinCAN.h"
#include <AP_Logger/AP_Logger.h>

static const uint16_t TINCAN_SEND_TIMEOUT_US = 500;

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

static const uint8_t CAN_IFACE_INDEX = 0;

// telemetry definitions
static const uint32_t TIN_CAN_ESC_UPDATE_MS = 100;

AP_TinCAN::AP_TinCAN()
{
    debug_can(2, "TinCAN: constructed\n\r");
}

AP_TinCAN *AP_TinCAN::get_tcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_TinCAN) {
        return nullptr;
    }
    return static_cast<AP_TinCAN*>(AP::can().get_driver(driver_index));
}

void AP_TinCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "TinCAN: starting init driver_index %d\n\r", driver_index);

    if (_initialized) {
        debug_can(1, "TinCAN: already initialized\n\r");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "TinCAN: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "TinCAN: mgr not initialized\n\r");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "TinCAN: no CAN driver\n\r");
        return;
    }

    // allocate array of clients, fixed maximum.
    
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_TinCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(1, "TinCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(2, "TinCAN: init done\n\r");

    return;
}

// loop to service CAN RX and let clients send
void AP_TinCAN::loop()
{
    //const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), TINCAN_SEND_TIMEOUT_US);

    uint32_t one_sec_ms = AP_HAL::millis() + 1000;

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(2, "TinCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        hal.scheduler->delay_microseconds(50);

        uavcan::CanFrame recv_frame;
        int now_ms = AP_HAL::millis();

        // ok this fuckin thing works good with a 50 ms select timeout, but not a one-second timeout
        // next, try using AP_HAL::micros64() again with the understanding that 50 ms or so is max 
        uint64_t next_us = (uint64_t)now_ms * 1000 + 50000;
        printf ( "pre read_frame: now_ms %d\n\r",  now_ms);
        uavcan::MonotonicTime timeout = uavcan::MonotonicTime::fromUSec(next_us);

        while (read_frame(recv_frame, timeout)) {
            printf ( "read_frame done pos\n\r" );

            dispatch_frame(CAN_IFACE_INDEX, recv_frame);

            // decode rpm and voltage data
            //if ((recv_frame.id >= MOTOR_DATA1) && (recv_frame.id <= MOTOR_DATA1 + 12)) {
        }
        
        now_ms = AP_HAL::millis();
        printf ( "post read_frame: now_ms %d\n\r",  now_ms);

        if ( one_sec_ms <= AP_HAL::millis() ) {
            one_sec_ms = AP_HAL::millis() + 1000;
            printf ( "It's been a second in %s next millis is %lu\n\r", __FUNCTION__, one_sec_ms );
        }
    }
}

// write frame on CAN bus
bool AP_TinCAN::write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to send command
    uavcan::CanSelectMasks inout_mask;
    do {
        inout_mask.read = 0;
        inout_mask.write = 1 << CAN_IFACE_INDEX;
        _select_frames[CAN_IFACE_INDEX] = &out_frame;
        _can_driver->select(inout_mask, _select_frames, timeout);

        // delay if no space is available to send
        if (!inout_mask.write) {
            hal.scheduler->delay_microseconds(50);
        }
    } while (!inout_mask.write);

    // send frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->send(out_frame, timeout, uavcan::CanIOFlagAbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_TinCAN::read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to read
    uavcan::CanSelectMasks inout_mask;
    inout_mask.read = 1 << CAN_IFACE_INDEX;
    inout_mask.write = 0;
    _select_frames[CAN_IFACE_INDEX] = &recv_frame;
    _can_driver->select(inout_mask, _select_frames, timeout);

    // return false if no data is available to read
    if (!inout_mask.read) {
        return false;
    }
    uavcan::MonotonicTime time;
    uavcan::UtcTime utc_time;
    uavcan::CanIOFlags flags {};

    // read frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->receive(recv_frame, time, utc_time, flags) == 1);
}

// called from SRV_Channels
void AP_TinCAN::update()
{
}

// send ESC telemetry messages over MAVLink
void AP_TinCAN::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
}

// iterate through client list
void AP_TinCAN::dispatch_frame( uint8_t interface_index, uavcan::CanFrame &recv_frame )
{
    bool consumed = false;
    
    for ( int i = 0; i < ARRAY_SIZE(client_array); i++ ) {
        if ( !consumed || client_array[i]->is_greedy(interface_index) ) {
            if ( client_array[i]->receive_frame(interface_index, recv_frame) ) {
                consumed = true;
            }
        }
    }
}

// client calls this to register with us
int AP_TinCAN::add_client(AP_TinCANClient * client)
{
    int i = 0;
    for ( ; i < ARRAY_SIZE(client_array); i++ ) {
        if ( !client_array[i] ) {
            client_array[i] = client;
            break;
        }
    }

    if ( i == ARRAY_SIZE(client_array) ) {
        return -1;
    }

    return 0;
}
#endif // HAL_WITH_UAVCAN
