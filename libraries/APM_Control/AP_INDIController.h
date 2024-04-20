#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AP_INDI/AP_INDI.h>
#include <RC_Channel/RC_Channel.h>

class AP_INDIController
{
public:
    AP_INDIController(const AP_FixedWing &parms);

    CLASS_NO_COPY(AP_INDIController);

    Vector3f get_servo_out_INDI(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw, Plane_Shape &plane_shape);

    void reset_INDI()
    {
        rate_indi.reset_filter();
    }

    void set_deflection(Vector3f deflection)
    {
        _indi_info.delta = deflection;
    }

    const AP_INDIInfo& get_indi_info(void) const
    {
        return _indi_info;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_FixedWing &aparm;
    AP_AutoTune::ATGains gains_roll;
    AP_AutoTune::ATGains gains_pitch;
    
    AP_Float _roll_ff_to_pitch;
    AP_Float _yaw_ff;

    AP_INDI rate_indi{5, 5, 5, 10, 10, 10, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    AP_INDIInfo _indi_info;

    Vector3f _get_att_out_INDI(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw);
    Vector3f _get_rate_out_INDI(Vector3f rate_desired, float airspeed, Plane_Shape &plane_shape);
    float _get_coordination_rate_offset_pitch(float &aspeed, bool &inverted) const;
};