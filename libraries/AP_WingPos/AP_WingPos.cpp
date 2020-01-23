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

    //_needVer = true;
    wingAngle = -90.0; // not possible...thus not yet init'd
    _handlingCalibrate = false;
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
/// if this report an angle of -90, then the reported value is not to be believed as it's not yet inited
float AP_WingPos::get_wa()
{
    return wingAngle;
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

// TBD: need to send the wing angle sensor readings out via MAVLink
void AP_WingPos::send_mavlink(uint8_t chan)
{
}

AP_WingPos *AP_WingPos::_singleton;

AP_WingPos &wingpos()
{
    return *AP_WingPos::instance();
}


// Manual control of wing position from a switch on the RC is controlled
// here. Manual control is not allowed when armed, since the autopilot
// controls it according to transition algo. (Is this even true?)
// In any case, that's not enforced here, it's enforced in the caller.
//
// There is a three-position switch with these meanings:
//  up/away from user - wing moves toward horizontal.
//  middle position - wing stops moving
//  down/toward user - wing movers toward vertical
//
// Initially, we don't move the wing if the switch is not in the middle
// position. The first move is not allowed until we see a transition
// from middle to not-middle. This way, the wing can't move unless
// the operator is actively directing it to.
//
void AP_WingPos::manual_control_wing_pos()
{
    if (_manualRcChan == 0) {
        // Manual control of wing disabled by configuration
        return;
    }

    int8_t rcChan = _manualRcChan - 1;

    AP_WASERVO_Direction direction = AP_WASERVO_DIRECTION_NONE;
    uint8_t speed = 0;

    uint16_t pulsewidth = RC_Channels::get_radio_in(rcChan);
    if (pulsewidth < 900 || pulsewidth > 2200) {
        return;
    }

    if (pulsewidth < 1000) {
        // switch up/away
        direction = AP_WASERVO_DIRECTION_WITHDRAW;
        speed = 100;

    } else if (pulsewidth > 2000) {
        // switch down/toward
        direction = AP_WASERVO_DIRECTION_EXTEND;
        speed = 100;
    }

    if (_rcWingMoveInitialHold) {
        if (_rcWingMovePrevDirection == AP_WASERVO_DIRECTION_NONE && direction != AP_WASERVO_DIRECTION_NONE) {
            // relase the initial hold
            _rcWingMoveInitialHold = false;
            _rcWingMovePrevDirection = direction;

        } else {
            // apply the hold by stomping on speed and direction, but first save the direction we
            // read from the switch
            _rcWingMovePrevDirection = direction;
            direction = AP_WASERVO_DIRECTION_NONE;
            speed = 0;
        }
    }

    // Push the values to the servo
    AP_WingAngleServo * waServo = AP_WingAngleServo::get_singleton();
    if (waServo) {
        waServo->set_output(direction, speed);
    }
}

// Called at 100 Hz or so. Can be used to do whatever WingPos needs to do
void AP_WingPos::periodic_activity()
{
    /* Should investigate whether this is efficient - what's the pattern for arm/disarm in
     * arbitrary client code anyway? How to avoid repetitive polling?
     * Also, do we want to allow manual control even when armed?
     */
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        manual_control_wing_pos();
    }
}
