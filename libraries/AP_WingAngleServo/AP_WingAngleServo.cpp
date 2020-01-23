#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef TINCAN_ENABLED
#include <AP_WingAngleServo/AP_WingAngleServo.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#define CAN_READ_16(buffer, offset) (be16toh(*(uint16_t *)&(buffer)[(offset)]))
#define CAN_WRITE_16(buffer, offset, value) ((buffer)[(offset)] = be16toh(value))

AP_WingAngleServo * AP_WingAngleServo::_singleton;

AP_WingAngleServo::AP_WingAngleServo()
{
    _singleton = this;
}

void AP_WingAngleServo::init()
{
    AP_TinCAN * tincan = AP_TinCAN::get_singleton();
    if (tincan) {
        //printf("%s: found tincan, adding us\r\n", __PRETTY_FUNCTION__);
        tincan->add_client(this);
        p_tincan = tincan;
    }
}

/* Haven't made the CAN Protocol for the servo yet, so this is a placeholder */
bool AP_WingAngleServo::receive_frame(uint8_t interface_index, const uavcan::CanFrame &recv_frame)
{
    union frame_id_t frame_id;
    frame_id.value = recv_frame.id;

    if (frame_id.object_address != 11 || frame_id.source_id != WING_ANGLE_SERVO_NODE_ID) {
        return false;
    }

    return true;
}

/* return true if we sent, false if not */
bool AP_WingAngleServo::transmit_slot(uint8_t interface_index)
{
    uint32_t now = AP_HAL::millis();
    if (now < next_pwm_val_send_ms) {
        return false;
    }

    /* Could make this protocol use the 11-bit header instead? */
    frame_id_t id = { {
            .object_address = SET_PWM_OBJ_ADDR,
            .destination_id = WING_ANGLE_SERVO_NODE_ID,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    uavcan::CanFrame frame { (id.value | uavcan::CanFrame::FlagEFF), nullptr, 0 };
    frame.data[0] = output_direction;
    frame.data[1] = output_speed;
    frame.dlc = 2;

    auto timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 800);

    bool xmitted = p_tincan->write_frame(frame, timeout);
    if (!xmitted) {
        if (++n_xmit_errors_in_burst > MAX_XMIT_RETRIES) {
            /* Give up on sending retries every millisecond */
            n_xmit_errors_in_burst = 0;
            next_pwm_val_send_ms += SEND_PWM_PERIOD_MS;
        }
    } else {
        /* schedule next send */
        next_pwm_val_send_ms += SEND_PWM_PERIOD_MS;
    }

    return true;
}

#endif

