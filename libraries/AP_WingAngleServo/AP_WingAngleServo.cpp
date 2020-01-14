#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifdef TINCAN_ENABLED
#include <AP_WingAngleServo/AP_WingAngleServo.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#define CAN_READ_16(buffer, offset) (be16toh(*(uint16_t *)&(buffer)[(offset)]))
#define CAN_WRITE_16(buffer, offset, value) ((buffer)[(offset)] = be16toh(value))

AP_WingAngleServo::AP_WingAngleServo() : output_pwm(0), next_pwm_val_send_ms(0)
{
}

void AP_WingAngleServo::init()
{
    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        AP_TinCAN * tincan = AP_TinCAN::get_tcan(i);
        if (tincan) {
            printf("%s: found tincan, adding us\r\n", __FUNCTION__);
            // client calls this to register with us
            tincan->add_client(this);
            p_tincan = tincan;
        }
    }
}

/* Haven't made the CAN Protocol for the servo yet, so this is a placeholder */
bool AP_WingAngleServo::receive_frame(uint8_t interface_index, uavcan::CanFrame &recv_frame)
{
    union frame_id_t frame_id;
    frame_id.value = recv_frame.id;

    if (frame_id.object_address != 11 && frame_id.source_id != WING_ANGLE_SERVO_NODE_ID) {
        return false;
    }

    return true;
}

/* return true if we sent, false if not */
bool AP_WingAngleServo::transmit_slot(uint8_t interface_index)
{
    uint32_t now = AP_HAL::millis();
    if ( now < next_pwm_val_send_ms ) {
        return false;
    }

#if 0
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        output_pwm = 0;
    }
#endif
    frame_id_t id = { {
            .object_address = SET_PWM_OBJ_ADDR,
            .destination_id = WING_ANGLE_SERVO_NODE_ID,
            .source_id = AUTOPILOT_NODE_ID,
            .priority = 0,
            .unused = 0
        }
    };

    //uint8_t can_data[2];
    //CAN_WRITE_16(can_data, 0, output_pwm);

    uavcan::CanFrame frame { (id.value), 0, 0 };
    CAN_WRITE_16(frame.data, 0, output_pwm);
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

