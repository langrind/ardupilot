#include "AP_WingPos.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_WingAngleServo/AP_WingAngleServo.h>
#include <AP_WingAngleSensor/AP_WingAngleSensor.h>

#include <GCS_MAVLink/GCS.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_WingPos::var_info[] = {
    // @Param: TYPE
    // @DisplayName: PWM Threshold
    // @Description: PWM Threshold for dial
    // @Values: 0-255
    // @User: Standard
    AP_GROUPINFO("_PWMT", 1, AP_WingPos, _pwmthreshold, 8),

    // @Param: TYPE
    // @DisplayName: debug msgs enabled
    // @Description: debug msgs enable
    // @Values: 0:no,1:yes
    // @User: Standard
    AP_GROUPINFO("_DBG", 2, AP_WingPos, _dbgmsg, 1),

    // @Param: TYPE
    // @DisplayName: test chute servo
    // @Description: enable testing of parachute servo
    // @Values: 0:no,1:yes
    // @User: Standard
    AP_GROUPINFO("_PCSRVTST", 3, AP_WingPos, _chuteservotest, 0),

    // @Param: TYPE
    // @DisplayName: min transition WA
    // @Description: lowest WA seen during transitions
    // @Values: 0-90
    // @User: Standard
    AP_GROUPINFO("_MINTWA", 4, AP_WingPos, _mintwa, 90),

    // @Param: TYPE
    // @DisplayName: max transition WA
    // @Description: largest WA seen during transition
    // @Values: 0-90
    // @User: Standard
    AP_GROUPINFO("_MAXTWA", 5, AP_WingPos, _maxtwa, 0),

    // @Param: TYPE
    // @DisplayName: actuator minimum
    // @Description: minimum PWM value given to WA actuator
    // @Values: 1-254
    // @User: Standard
    AP_GROUPINFO("_ACTMIN", 6, AP_WingPos, _actmin, 0),

    // @Param: TYPE
    // @DisplayName: departure transition WA
    // @Description: the WA appropriate for departure transition
    // @Values: 0-90
    // @User: Standard
    AP_GROUPINFO("_DEPTWA", 7, AP_WingPos, _deptwa, 90),

    // @Param: TYPE
    // @DisplayName: arrival transition WA
    // @Description: the WA appropriate for arrival transition
    // @Values: 0-90
    // @User: Standard
    AP_GROUPINFO("_ARRTWA", 8, AP_WingPos, _arrtwa, 90),

    // @Param: TYPE
    // @DisplayName: Wing Angle Control Switch
    // @Description: RC Channel to use to manually control wing angle
    // @User: Standard
    AP_GROUPINFO("_MANRC_CH", 9, AP_WingPos, _manualRcChan, 7),

    // @Param: TYPE
    // @DisplayName: Wing Angle Left Sensor Max
    // @Description: Max value of left sensor observed during cal
    // @User: Standard
    AP_GROUPINFO("_LMAX", 10, AP_WingPos, _leftSensorMax, 0),

    // @Param: TYPE
    // @DisplayName: Wing Angle Left Sensor Min
    // @Description: Min value of left sensor observed during cal
    // @User: Standard
    AP_GROUPINFO("_LMIN", 11, AP_WingPos, _leftSensorMin, 0),

    // @Param: TYPE
    // @DisplayName: Wing Angle Right Sensor Max
    // @Description: Max value of right sensor observed during cal
    // @User: Standard
    AP_GROUPINFO("_RMAX", 12, AP_WingPos, _rightSensorMax, 0),

    // @Param: TYPE
    // @DisplayName: Wing Angle Right Sensor Min
    // @Description: Min value of right sensor observed during cal
    // @User: Standard
    AP_GROUPINFO("_RMIN", 13, AP_WingPos, _rightSensorMin, 0),

    // @Param: TYPE
    // @DisplayName: Wing Angle Calibration Switch
    // @Description: RC Channel used to start/stop Wing Angle calibration
    // @User: Standard
    AP_GROUPINFO("_CALRC_CH", 14, AP_WingPos, _calRcChan, 6),

    AP_GROUPEND
};

static struct param_name_map p_n_m[] = {
  {"airspeed", WA_AIRSPEED_PARAM, WA_TYPE_FLOAT},
  {"isarmed", WA_ISARMED_PARAM, WA_TYPE_BOOL},
  {"isforcedflying", WA_ISFORCEDFLYING_PARAM, WA_TYPE_BOOL},
  {"ctrlmode", WA_CTRLMODE_PARAM, WA_TYPE_INT},
  {0} // list terminator
};


void
AP_WingPos::init()
{
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // check to see if _mintwa/_maxtwa/_deptwa have been init'd thru the param file; bail if not
    if (_mintwa == 90 || _maxtwa == 0 || _deptwa == 90) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "WA MINTWA and MAXTWA and DEPTWA *must* be set as params");
        return;
    }
#endif
}

/// Constructor
AP_WingPos::AP_WingPos()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton == nullptr) {
        _singleton = this;
    }
    init();
}

/// recalibrate_wasensors - force a recalibration of the WA system
void AP_WingPos::recalibrate_wasensors()
{
}

/// recalibrate_all - force a recalibration of the WA system and RC dial...
void AP_WingPos::recalibrate_all()
{
}

/// wingpos_check - are we allowed to arm? Yes iff current state is ready for an input (and not calibrating)
bool AP_WingPos::wingpos_checks(bool display_failure)
{
    return false;
}

/// set_wing_to - set the wing angle to the specified angle (0..90)
void AP_WingPos::set_wing_to(float wa)
{
}


/// get_preset_wa - return one of two preset wing angles. selector == 0 ==> airplane angle, 1 ==> begin of departure transition angle
int AP_WingPos::get_preset_wa(enum wa_presets selector)
{
    switch (selector)
    {
      case WA_DOWN:
        return _mintwa; // wing down
      case WA_DEPARTURE_TRANSITION:
        return _deptwa; // wing position for departure transition
      case WA_ARRIVAL_TRANSITION:
        return _arrtwa; // wing position for arrival transition
      case WA_UP:
        return _maxtwa; // wing up
    }
    return 0;  // NOTREACHED
}

/// get_wa - return currently reported wing angle
float AP_WingPos::get_wa()
{
    float degrees = INVALID_WP_DEGREES;
    AP_WingAngleSensor *sensor = AP_WingAngleSensor::get_singleton();
    if (sensor) {
        _left_sensor_value = sensor->get_left_sensor_val();
        _right_sensor_value = sensor->get_right_sensor_val();
        degrees = calculate_wing_angle();
    }
    return degrees;
}

/// find_m_n_p - given a wa_param value, return the corresponding element (if any) of the p_n_m[] array.
struct param_name_map *AP_WingPos::find_m_n_p(enum wa_param param)
{
    struct param_name_map *p;

    for (p = p_n_m; p->name != nullptr; p++)
        if (p->p == param)
            return p;
    return nullptr;
}

/// get_param - return value of identified param
bool AP_WingPos::get_param(enum wa_param param, float *fp)
{
    struct param_name_map *p;

    p = find_m_n_p(param);
    if (p == nullptr)
        return false;
    if (!p->inited)
        return false;
    switch (p->p)
    {
      case WA_AIRSPEED_PARAM:
        if (fp != nullptr)
            *fp = p->u.f;
        break;
      default:
        return false;
    }
    return true;
}

/// get_param - return value of identified param
bool AP_WingPos::get_param(enum wa_param param, int *ip)
{
    struct param_name_map *p;

    p = find_m_n_p(param);
    if (p == nullptr)
        return false;
    if (!p->inited)
        return false;
    switch (p->p)
    {
      case WA_CTRLMODE_PARAM:
        if (ip != nullptr)
            *ip = p->u.i;
        break;
      default:
        return false;
    }
    return true;
}

/// get_param - return value of identified param
bool AP_WingPos::get_param(enum wa_param param, bool *bp)
{
    struct param_name_map *p;

    p = find_m_n_p(param);
    if (p == nullptr)
        return false;
    if (!p->inited)
        return false;
    switch (p->p)
    {
      case WA_ISFORCEDFLYING_PARAM:
      case WA_ISARMED_PARAM:
        if (bp != nullptr)
            *bp = p->u.b;
        break;
      default:
        return false;
    }
    return true;
}

// send the wing angle sensor readings out via MAVLink
void AP_WingPos::send_mavlink(mavlink_channel_t chan)
{
    AP_WingAngleSensor *sensor = AP_WingAngleSensor::get_singleton();
    if (sensor) {
        _left_sensor_value = sensor->get_left_sensor_val();
        _right_sensor_value = sensor->get_right_sensor_val();
        float degrees = calculate_wing_angle();
        mavlink_msg_wing_angle_send(chan, degrees);
    }
}

AP_WingPos *AP_WingPos::_singleton;

AP_WingPos &wingpos()
{
    return *AP_WingPos::instance();
}


void AP_WingPos::drive_wing_pos(AP_WASERVO_Direction direction, uint8_t speed)
{
    // Push the values to the servo object, which will send them on its
    // own schedule, asynchronous to this update
    AP_WingAngleServo * waServo = AP_WingAngleServo::get_singleton();
    if (waServo) {
        waServo->set_output(direction, speed);
    }
}

void AP_WingPos::drive_wing_pos_horizontal()
{
    drive_wing_pos(AP_WASERVO_DIRECTION_RETRACT, 100);
}

void AP_WingPos::drive_wing_pos_vertical()
{
    drive_wing_pos(AP_WASERVO_DIRECTION_EXTEND, 100);
}

void AP_WingPos::drive_wing_pos_stop()
{
    drive_wing_pos(AP_WASERVO_DIRECTION_NONE, 0);
}

// This needs improvement. The values are chosen by observation
// of what happens with my FrSky controller. I don't know how
// to choose values that are guaranteed to be correct if something
// should change, and I don't know how to choose values such that
// signal errors are mininimized
AP_WingPos_SwitchPosition AP_WingPos::read_switch(int8_t rcChan)
{
    if (rcChan == 0) {
        // The switch is not enabled by configuration
        return WP_SWITCH_INVALID;
    }

    uint16_t pulsewidth = RC_Channels::get_radio_in(rcChan-1);
    if (pulsewidth < 900 || pulsewidth > 2200) {
        return WP_SWITCH_INVALID;
    }

    if (pulsewidth < 1000) {
        // switch up/away
        return WP_SWITCH_TOP;

    } else if (pulsewidth > 2000) {
        // switch down/toward
        return WP_SWITCH_BOTTOM;
    }
    return WP_SWITCH_MIDDLE;
}

void AP_WingPos::cancel_calibration()
{
    drive_wing_pos_stop();
    _calState = WP_CAL_INIT;
    _manualControlState = AP_WINGPOS_MC_STATE_CALIBRATION_FINISHED;
}

void AP_WingPos::complete_calibration()
{
    cancel_calibration();

    // Store the extreme sensor readings in their respective parameters
    enum ap_var_type var_type = AP_PARAM_INT16;
    AP_Param * pParam = AP_Param::find("WINGPOS_LMAX", &var_type, NULL);
    if (pParam) {
        pParam->save_sync();
        printf("Wrote WINGPOS_LMAX\n\r");
    } else {
        printf("Did not find WINGPOS_LMAX\n\r");
    }

    pParam = AP_Param::find("WINGPOS_RMAX", &var_type, NULL);
    if (pParam) {
        pParam->save_sync();
        printf("Wrote WINGPOS_RMAX\n\r");
    } else {
        printf("Did not find WINGPOS_RMAX\n\r");
    }

    pParam = AP_Param::find("WINGPOS_LMIN", &var_type, NULL);
    if (pParam) {
        pParam->save_sync();
        printf("Wrote WINGPOS_LMIN\n\r");
    } else {
        printf("Did not find WINGPOS_LMIN\n\r");
    }

    pParam = AP_Param::find("WINGPOS_RMIN", &var_type, NULL);
    if (pParam) {
        pParam->save_sync();
        printf("Wrote WINGPOS_RMIN\n\r");
    } else {
        printf("Did not find WINGPOS_RMIN\n\r");
    }

    // Force the parameters to written to EEPROM


}

void AP_WingPos::start_calibration()
{
    AP_WingAngleSensor *sensor = AP_WingAngleSensor::get_singleton();
    if (!sensor) {
        printf ( "Failed to get AP_WingAnglesensor instance\n\r");
        cancel_calibration();
    }

    _left_sensor_value = sensor->get_left_sensor_val();
    _right_sensor_value = sensor->get_right_sensor_val();
    _lastSensorChangeTime = AP_HAL::millis();

    drive_wing_pos(AP_WASERVO_DIRECTION_EXTEND, 100);
    _calState = WP_CAL_EXTEND;
}

void AP_WingPos::record_extreme_extension_sensor_readings()
{
    // need param interface
    _leftSensorMax = _left_sensor_value;
    _rightSensorMax = _right_sensor_value;

    printf("Extension values: L:%d R:%d\n\r", _left_sensor_value, _right_sensor_value);
}

void AP_WingPos::record_extreme_retraction_sensor_readings()
{
    // need param interface
    _leftSensorMin = _left_sensor_value;
    _rightSensorMin = _right_sensor_value;

    printf("Retraction values: L:%d R:%d\n\r", _left_sensor_value, _right_sensor_value);
}

uint16_t AP_WingPos::raw_sensor_difference(uint16_t val1, uint16_t val2)
{
    if (val1 > val2) {
        return val1 - val2;
    }
    return val2 - val1;
}

// return true if sensor readings have changed, false if they havent
bool AP_WingPos::process_sensor_readings()
{
    AP_WingAngleSensor *sensor = AP_WingAngleSensor::get_singleton();
    if (!sensor) {
        printf ( "Failed to get AP_WingAnglesensor instance\n\r");
        cancel_calibration();
        return true;
    }

    uint16_t left_sensor_val = sensor->get_left_sensor_val();
    uint16_t right_sensor_val = sensor->get_right_sensor_val();

    uint16_t left_diff = raw_sensor_difference(left_sensor_val, _left_sensor_value);
    uint16_t right_diff = raw_sensor_difference(right_sensor_val, _right_sensor_value);

    _left_sensor_value = left_sensor_val;
    _right_sensor_value = right_sensor_val;

    if (left_diff > AP_WINGPOS_SENSOR_EPSILON || right_diff > AP_WINGPOS_SENSOR_EPSILON) {
        return true;
    }

    return false;
}

// * extend the wing until the angle readings stay unchanged for 2 seconds
//   * should implement a failure timeout?
// * save the readings as max extension
// * retract until the angle readings stay unchanged for 2 seconds
// * save the readings as min extension
// * write the readings to their parameters and force the parameters to be written to eeprom
//
void AP_WingPos::calibration_state_machine()
{
    uint32_t now = AP_HAL::millis();
    if (now < _calTickMs + WP_CAL_TICK_MS) {
        return;
    }

    _calTickMs = WP_CAL_TICK_MS;

    switch (_calState) {
    case WP_CAL_INIT:
        return;
    case WP_CAL_EXTEND:
        {
            /* get and record sensor readings, check if they have changed */
            if (process_sensor_readings()) {
                _lastSensorChangeTime = now;
            } else if (now > _lastSensorChangeTime + WP_CAL_SENSOR_UNCHANGED_TIME_MS) {
                record_extreme_extension_sensor_readings();
                _calState = WP_CAL_RETRACT;
                _lastSensorChangeTime = AP_HAL::millis();
                drive_wing_pos(AP_WASERVO_DIRECTION_RETRACT, 100);
            }
        }
        break;
    case WP_CAL_RETRACT:
        {
            /* get and record sensor readings, check if they have changed */
            if (process_sensor_readings()) {
                _lastSensorChangeTime = now;
            } else if (now > _lastSensorChangeTime + WP_CAL_SENSOR_UNCHANGED_TIME_MS) {
                record_extreme_retraction_sensor_readings();
                complete_calibration();
            }
        }
        break;
    }
}

//
// Manual Control State Machine responds to two 3-position switches, allowing the user to:
// 1) Move the wing up or down with the "man" switch
// 2) Stop moving the wing with the "man" switch
// 3) When disarmed, enter calibration mode with the "cal" switch
// 4) Exit calibration mode with the "cal" switch or the man switch
//
// States:
// * INIT - start in this state
// * STARTUP_HOLDDOWN - making sure we don't drive the wing with previous switch position.
// * NOT_DRIVING - stopped, but ready to start moving if the user switches the switch
// * DRIVING - driving one way or the other
// * CALIBRATING - calibrating
// * CALIBRATION_DONE - calibration is done, waiting for the user to move the switch(es) back to the
//   middle
//
// This routine is called periodically with the latest switch values. Therefore to
// simplify the state machine (at least when disarmed) we can always stop wing movement
// and return to init state. The user will have to return both switches to middle position
// before any further manual wing pos control can be exerted
void AP_WingPos::manual_control_switches_state_machine(AP_WingPos_SwitchPosition curSwPos,
                                                       AP_WingPos_SwitchPosition curCalSwPos,
                                                       bool disarmed)
{
    if (disarmed) {
        switch (_manualControlState) {
        case AP_WINGPOS_MC_STATE_INIT:
        case AP_WINGPOS_MC_STATE_STARTUP_HOLDDOWN:
            if (curCalSwPos == WP_SWITCH_MIDDLE && curSwPos == WP_SWITCH_MIDDLE ) {
                _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
            } else {
                _manualControlState = AP_WINGPOS_MC_STATE_STARTUP_HOLDDOWN;
            }
            break;

        case AP_WINGPOS_MC_STATE_NOT_DRIVING:
            if ((curCalSwPos == _lastCalSwitchPos || curCalSwPos == WP_SWITCH_INVALID)  &&
                (curSwPos == _lastSwitchPos || curSwPos == WP_SWITCH_INVALID)) {
                break;
            }

            // a switch has changed - in disarmed state we give priority to cal switch
            if (curCalSwPos != _lastCalSwitchPos) {
                switch (curCalSwPos) {
                case WP_SWITCH_BOTTOM:
                case WP_SWITCH_TOP:
                    start_calibration();
                    _manualControlState = AP_WINGPOS_MC_STATE_CALIBRATING;
                    break;
                case WP_SWITCH_MIDDLE:
                case WP_SWITCH_INVALID:
                    // unexpected
                    drive_wing_pos_stop();
                    _manualControlState = AP_WINGPOS_MC_STATE_INIT;
                    break;
                }
            } else {
                switch (curSwPos) {
                case WP_SWITCH_BOTTOM:
                    drive_wing_pos_horizontal();
                    _manualControlState = AP_WINGPOS_MC_STATE_DRIVING;
                    break;
                case WP_SWITCH_TOP:
                    drive_wing_pos_vertical();
                    _manualControlState = AP_WINGPOS_MC_STATE_DRIVING;
                    break;
                case WP_SWITCH_MIDDLE:
                    drive_wing_pos_stop();
                    _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
                    break;
                case WP_SWITCH_INVALID:
                    drive_wing_pos_stop();
                    _manualControlState = AP_WINGPOS_MC_STATE_INIT;
                    break;
                }
            }
            break;

        case AP_WINGPOS_MC_STATE_DRIVING:
            // if we are driving the wing with man switch, ignore cal switch
            // if man switch hasn't changed, nothing to do
            if (curSwPos == _lastSwitchPos) {
                break;
            }

            // In this state, if the switch changed in any way, we are stopping the wing
            drive_wing_pos_stop();

            switch (curSwPos) {
            case WP_SWITCH_TOP:
            case WP_SWITCH_BOTTOM:
            case WP_SWITCH_INVALID:
                // bad input or somehow we skipped the middle position, so go into init state - user
                // must return both switches to middle before further switch input accepted
                _manualControlState = AP_WINGPOS_MC_STATE_INIT;
                break;
            case WP_SWITCH_MIDDLE:
                _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
                break;
            }
            break;

        case AP_WINGPOS_MC_STATE_CALIBRATING:
            if (curSwPos != _lastSwitchPos || curCalSwPos != _lastCalSwitchPos ) {
                cancel_calibration();
                _manualControlState = AP_WINGPOS_MC_STATE_INIT;
            }
            break;

        case AP_WINGPOS_MC_STATE_CALIBRATION_FINISHED:
            if (curSwPos == WP_SWITCH_MIDDLE) {
                _manualControlState = AP_WINGPOS_MC_STATE_INIT;
            }
            break;
        }
    } else {
        switch (_manualControlState) {
        case AP_WINGPOS_MC_STATE_INIT:
        case AP_WINGPOS_MC_STATE_STARTUP_HOLDDOWN:
            if (curSwPos == WP_SWITCH_MIDDLE ) {
                _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
            } else {
                _manualControlState = AP_WINGPOS_MC_STATE_STARTUP_HOLDDOWN;
            }
            break;

        case AP_WINGPOS_MC_STATE_NOT_DRIVING:
            if (curSwPos == _lastSwitchPos || curSwPos == WP_SWITCH_INVALID) {
                break;
            }

            // man switch has changed
            switch (curSwPos) {
            case WP_SWITCH_BOTTOM:
                drive_wing_pos_horizontal();
                _manualControlState = AP_WINGPOS_MC_STATE_DRIVING;
                break;
            case WP_SWITCH_TOP:
                drive_wing_pos_vertical();
                _manualControlState = AP_WINGPOS_MC_STATE_DRIVING;
                break;
            case WP_SWITCH_MIDDLE:
                drive_wing_pos_stop();
                _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
                break;
            case WP_SWITCH_INVALID:
                drive_wing_pos_stop();
                _manualControlState = AP_WINGPOS_MC_STATE_STARTUP_HOLDDOWN;
                break;
            }
            break;

        case AP_WINGPOS_MC_STATE_DRIVING:
            // if man switch hasn't changed, nothing to do
            if (curSwPos == _lastSwitchPos) {
                break;
            }

            // No matter what happened with the switch, we are stopping the wing
            drive_wing_pos_stop();

            switch (curSwPos) {
            case WP_SWITCH_TOP:
            case WP_SWITCH_BOTTOM:
            case WP_SWITCH_INVALID:
                // bad input or somehow we skipped the middle position, so go into init state - user
                // must return switch to middle before further switch input accepted
                _manualControlState = AP_WINGPOS_MC_STATE_INIT;
                break;
            case WP_SWITCH_MIDDLE:
                _manualControlState = AP_WINGPOS_MC_STATE_NOT_DRIVING;
                break;
            }
            break;

        case AP_WINGPOS_MC_STATE_CALIBRATING:
            // We can't be here, and yet here we are. Stop calibrating
            cancel_calibration();
            _manualControlState = AP_WINGPOS_MC_STATE_INIT;
            break;

        case AP_WINGPOS_MC_STATE_CALIBRATION_FINISHED:
            _manualControlState = AP_WINGPOS_MC_STATE_INIT;
            break;
        }
    }
}

// Manual control of wing position from switches on the RC is controlled
// here. Manual control is allowed when armed, but not calibration.
// Have yet to work out the details of making manual and auto-control
// coexist.
//
// There is a three-position switch with these meanings:
//  up/away from user - wing moves toward horizontal.
//  middle position - wing stops moving
//  down/toward user - wing movers toward vertical
//
// Another three-position switch with these meanings:
//  up/away from user - start cal
//  middle position - stop cal
//  down/toward user - start cal
//
// Initially, we don't move the wing if the switches aren't in the middle
// position. The first move is not allowed until we see a transition
// from middle to not-middle. This way, the wing can't move unless
// the operator is actively directing it to.
//
void AP_WingPos::manual_control_wing_pos(bool disarmed)
{
    AP_WingPos_SwitchPosition curSwPos    = read_switch(_manualRcChan);
    AP_WingPos_SwitchPosition curCalSwPos = disarmed ? read_switch(_calRcChan) : WP_SWITCH_INVALID;

    manual_control_switches_state_machine(curSwPos, curCalSwPos, disarmed);

    _lastSwitchPos = curSwPos;
    _lastCalSwitchPos = curCalSwPos;
}

float AP_WingPos::calculate_wing_angle()
{
    float left_degrees = INVALID_WP_DEGREES;
    float right_degrees = INVALID_WP_DEGREES;

    if (_leftSensorMax && _leftSensorMin && _leftSensorMax != _leftSensorMin) {
        float ratio = (float)(_left_sensor_value - _leftSensorMin) /
            (float)(_leftSensorMax - _leftSensorMin);
        left_degrees = ratio * 90.0;
    }

    if (_rightSensorMax && _rightSensorMin && _rightSensorMax != _rightSensorMin) {
        float ratio = (float)(_right_sensor_value - _rightSensorMin) /
            (float)(_rightSensorMax - _rightSensorMin);
        right_degrees = ratio * 90.0;
    }

    if (left_degrees >= INVALID_WP_DEGREES) {
        return right_degrees;
    } else if (right_degrees >= INVALID_WP_DEGREES) {
        return left_degrees;
    }

    return (left_degrees + right_degrees) / 2;
}

// Called at 100 Hz or so. (For now, 10 Hz) Can be used to do whatever WingPos needs to do
void AP_WingPos::periodic_activity()
{
    /* Should investigate whether this is efficient - what's the pattern for arm/disarm in
     * arbitrary client code anyway? How to avoid repetitive polling?
     * Also, do we want to allow manual control even when armed? I think we do, and
     * we should incorporate safety_switch_state() into our state machine.
     */
    bool disarmed = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
    manual_control_wing_pos(disarmed);

    if (_manualControlState == AP_WINGPOS_MC_STATE_CALIBRATING) {
        calibration_state_machine();
    }
}
