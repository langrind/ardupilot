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
 * AP_Alt_KDECAN.cpp
 *
 *      Author: Francisco Ferreira
 *
 * Hacked up by Nik Langrind to experiment with different CAN Mux layer
 * This alternative implementation only supports telemetry, and doesn't
 * support dynamic enumeration
 */


#include <AP_HAL/AP_HAL.h>

#ifdef TINCAN_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_Alt_KDECAN.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

AP_Alt_KDECAN::AP_Alt_KDECAN()
{
    debug_can(2, "ALT_KDECAN: constructed\n\r");

    transmit_period_ms = 0;
    transmit_offset_ms = 0;
}

void AP_Alt_KDECAN::init()
{
    debug_can(2, "ALT_KDECAN: starting init\n\r");

    if (_initialized) {
        debug_can(1, "ALT_KDECAN: already initialized\n\r");
        return;
    }

    AP_TinCAN * tincan = AP_TinCAN::get_singleton();
    if (tincan) {
        //printf("%s: found tincan, adding us\r\n", __PRETTY_FUNCTION__);
        tincan->add_client(this);
        p_tincan = tincan;
    }
    _initialized = true;

    next_send_ms = AP_HAL::millis() + TELEMETRY_REQUEST_PERIOD_MS;
    next_send_destination_id = next_send_destination_id == ESC_NODE_ID_FIRST;

    debug_can(2, "ALT_KDECAN: init done\n\r");

    return;
}

#define CAN_READ_16(buffer, offset) (be16toh(*(uint16_t *)&(buffer)[offset]))

/*
 * This is the receive routine for KDE ESCs when using the TinCAN framework.
 * With Tincan, this routine may be offered frames that are not KDE frames.
 * We have to recognize those and return false, (without messing up the
 * frame). If they are KDE frames, we process them and return true.
 */
bool AP_Alt_KDECAN::receive_frame(uint8_t interface_index, const uavcan::CanFrame &recv_frame)
{
    union frame_id_t frame_id;
    frame_id.value = recv_frame.id;

    if (frame_id.destination_id != AUTOPILOT_NODE_ID ||
        frame_id.source_id < ESC_NODE_ID_FIRST ||
        frame_id.source_id >= (KDECAN_MAX_NUM_ESCS + ESC_NODE_ID_FIRST)) {
        return false;
    }

    switch (frame_id.object_address) {
    case TELEMETRY_OBJ_ADDR: {
        if (recv_frame.dlc != 8) {
            /* invalid length */
            break;
        }

        if (!_telem_sem.take(1)) {
            debug_can(2, "ALT_KDECAN: failed to get telemetry semaphore on write\n\r");
            break;
        }

        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].time = AP_HAL::micros64();
        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].voltage = CAN_READ_16(recv_frame.data, 0);
        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].current = CAN_READ_16(recv_frame.data, 2);
        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].rpm = CAN_READ_16(recv_frame.data, 4);
        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].temp = recv_frame.data[6];
        _telemetry[frame_id.source_id - ESC_NODE_ID_FIRST].new_data = true;
        _telem_sem.give();
        break;
    }
    default:
        // discard frame
        break;
    }
    return true;
}

/*
 * KDECAN calls us once per millisecond. Check if we are ready to send, if so, do so.
 * return true if we sent, false if not
 *
 * Unlike the real KDECAN code, we only send the get-telemetry message, which
 * is the message sent with object address=11 (TELEMETRY_OBJ_ADDR)
 *
 * We send 10 of these messages per second to each KDE ESC.
  union frame_id_t id;
  CAN_message_t message;

  id.object_address = 11;
  id.destination_id = dest;
  id.source_id = 0;
  id.priority = 0;

  message.id = id.value;
  message.len = 0;
  message.ext = 1;

 */
bool AP_Alt_KDECAN::transmit_slot(uint8_t interface_index)
{
    uint32_t now = AP_HAL::millis();
    if ( now < next_send_ms ) {
        return false;
    }

    frame_id_t id = { {
            .object_address = TELEMETRY_OBJ_ADDR,
            .destination_id = next_send_destination_id,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    uavcan::CanFrame frame { (id.value | uavcan::CanFrame::FlagEFF), nullptr, 0 };

    auto timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 800);

    bool xmitted = p_tincan->write_frame(frame, timeout);
    if (!xmitted) {
        /* we don't retry or anything, just count and move on */
        ++n_xmit_errors;
    }

    /* schedule next send */
    next_send_ms = now + TELEMETRY_REQUEST_PERIOD_MS;
    next_send_destination_id++;
    if ( next_send_destination_id >= ESC_NODE_ID_FIRST + KDECAN_MAX_NUM_ESCS ) {
        next_send_destination_id = ESC_NODE_ID_FIRST;
    }
    return true;
}

#define DEFAULT_NUM_POLES 14

// send ESC telemetry messages over MAVLink
void AP_Alt_KDECAN::send_esc_telemetry_mavlink(uint8_t chan)
{
    if (!_telem_sem.take(1)) {
        debug_can(2, "ALT_KDECAN: failed to get telemetry semaphore on MAVLink read\n\r");
        return;
    }

    telemetry_info_t telem_buffer[KDECAN_MAX_NUM_ESCS];
    memcpy(telem_buffer, _telemetry, sizeof(telemetry_info_t) * KDECAN_MAX_NUM_ESCS);
    _telem_sem.give();

    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t rpm[4] {};
    uint8_t temperature[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t count[4] {};
    uint8_t num_poles = DEFAULT_NUM_POLES;
    uint64_t now = AP_HAL::micros64();

    for (uint8_t i = 0; i < KDECAN_MAX_NUM_ESCS && i < 8; i++) {
        uint8_t idx = i % 4;
        if (telem_buffer[i].time && (now - telem_buffer[i].time < 1000000)) {
            voltage[idx]      = telem_buffer[i].voltage;
            current[idx]      = telem_buffer[i].current;
            rpm[idx]          = uint16_t(telem_buffer[i].rpm  * 60UL * 2 / num_poles);
            temperature[idx]  = telem_buffer[i].temp;
        } else {
            voltage[idx] = 0;
            current[idx] = 0;
            rpm[idx] = 0;
            temperature[idx] = 0;
        }

        if (idx == 3 || i == KDECAN_MAX_NUM_ESCS - 1) {
            if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, ESC_TELEMETRY_1_TO_4)) {
                return;
            }

            if (i < 4) {
                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)chan, temperature, voltage, current, totalcurrent, rpm, count);
            } else {
                mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)chan, temperature, voltage, current, totalcurrent, rpm, count);
            }
        }
    }
}

#endif // TINCAN_ENABLED
