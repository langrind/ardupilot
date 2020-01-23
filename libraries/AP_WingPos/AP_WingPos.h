/// @file	AP_WingPos.h
/// @brief	Wing position library
#pragma once


#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_WingAngleServo/AP_WingAngleServo.h>

#define WINGPOSMAJOR        1
#define WINGPOSMINOR        0

enum wa_type {
    WA_TYPE_FLOAT,
    WA_TYPE_BOOL,
    WA_TYPE_INT
};

enum wa_param {
    WA_AIRSPEED_PARAM,
    WA_ISARMED_PARAM,
    WA_ISFORCEDFLYING_PARAM,
    WA_CTRLMODE_PARAM
};

enum wa_presets {
    WA_DOWN,
    WA_DEPARTURE_TRANSITION,
    WA_ARRIVAL_TRANSITION,
    WA_UP
};

struct param_name_map {
    const char *name;
    enum wa_param p;
    enum wa_type t;
    union {
      float f;
      int i;
      bool b;
    } u;
    bool inited;
};

/// @class	AP_WingPos
/// @brief	Class managing the wing position
class AP_WingPos {

public:
    /// Constructor
    AP_WingPos();

    static class AP_WingPos *instance() {
        return _singleton;
    }

    /* Do not allow copies */
    AP_WingPos(const AP_WingPos &other) = delete;
    AP_WingPos &operator=(const AP_WingPos&) = delete;

    void periodic_activity();

    /// recalibrate_wasensors - tell the WA system set to recalibrate the WA sensors (only when unarmed)
    void recalibrate_wasensors();

    /// recalibrate_all - tell the WA system to recalibrate the WA sensors and RC dial (only when unarmed)
    void recalibrate_all();

    /// wingpos_checks - indicate if it's OK to arm the AC
    bool wingpos_checks(bool);

    /// send_mavlink() - send wing angle info over mavlink - need to find a mavlink message to hold it
    void send_mavlink(uint8_t);

    /// set_wing_to - set the wing angle to the specified angle (0..90)
    void set_wing_to(float);

    /// get_preset_wa - return one of two preset wing angles
    int get_preset_wa(enum wa_presets);

    /// get_wa - return currently reported wing angle
    float get_wa();

    /// get_param - return value of identified param
    bool get_param(enum wa_param, float *);

    /// get_param - return value of identified param
    bool get_param(enum wa_param, int *);

    /// get_param - return value of identified param
    bool get_param(enum wa_param, bool *);

    AP_Float wingAngle;                    // currently reported wing angle

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_WingPos *_singleton;

    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _msg_on_RC;     // verbosity level of msgs which should be sent to RC
    AP_Int8     _pwmthreshold;  // threshold for changes in PWM value from dial before we "pay attention"
    AP_Int8     _dbgmsg;        // debugging msgs enabled
    AP_Int8     _chuteservotest;// enable flag for parachute servo testing
    AP_Int8     _mintwa;        // minimum WA seen in transitions (must be set in params)
    AP_Int8     _deptwa;        // departure transition WA (must be set in params)
    AP_Int8     _arrtwa;        // arrival transition WA (must be set in params)
    AP_Int8     _maxtwa;        // maximum WA seen in transitions (must be set in params)
    AP_Int16    _actmin;        // minumum PWM value for WA actuator
    AP_Int16    _ctrlmodeswtch; // initial value of Control Mode Switch (for use when testing w/o an RC)
    AP_Int8     _manualRcChan;  // RC Channel to use to manually control wing angle

    // funcs

    /// find_m_n_p - given a wa_param value, return the corresponding element (if any) of the p_n_m[] array.
    struct param_name_map *find_m_n_p(enum wa_param);

    /// init - initialize internal stuff
    void        init();
    void        manual_control_wing_pos();

    // internal variables

    bool        _handlingCalibrate : 1;     // does the current set of deferred contain a calibration?
    bool                 _rcWingMoveInitialHold = true;
    AP_WASERVO_Direction _rcWingMovePrevDirection = AP_WASERVO_DIRECTION_EXTEND;

};

AP_WingPos &wingpos();
