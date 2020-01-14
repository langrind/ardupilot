#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef TINCAN_ENABLED
#include <AP_WingAngleSensor/AP_WingAngleSensor.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#define CAN_READ_16(buffer, offset) (be16toh(*(uint16_t *)&(buffer)[offset]))

AP_WingAngleSensor::AP_WingAngleSensor()
{
    transmit_period_ms = 0;
    transmit_offset_ms = 0;
}

void AP_WingAngleSensor::init()
{
    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        AP_TinCAN * tincan = AP_TinCAN::get_tcan(i);
        if (tincan) {
            printf("%s: found tincan, adding us\r\n", __FUNCTION__);
            // client calls this to register with us
            tincan->add_client(this);
            break;
        }
    }
}

bool AP_WingAngleSensor::receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame)
{
    union frame_id_t frame_id;
    frame_id.value = recv_frame.id;

    if (frame_id.object_address != 11 && frame_id.source_id != 0x69) {
        return false;
    }

    left_sensor_val = CAN_READ_16(recv_frame.data, 0);
    right_sensor_val = CAN_READ_16(recv_frame.data, 2);
    last_sensor_update_us = AP_HAL::micros64();
    return true;
}

#endif
