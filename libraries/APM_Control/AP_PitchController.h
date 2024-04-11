#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_INDI/AP_INDI.h>
#include <RC_Channel/RC_Channel.h>

class AP_PitchController
{
public:
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode, Matrix3f &indi_gains,
                        AP_Plane_Shape &plane_shape, INDI_KF_Params &indi_kf_params, RC_Channel::AuxSwitchPos &sw_pos, uint8_t &flag);

    // setup a one loop FF scale multiplier. This replaces any previous scale applied
    // so should only be used when only one source of scaling is needed
    void set_ff_scale(float _ff_scale) { ff_scale = _ff_scale; }

    void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }

    void autotune_start(void);
    void autotune_restore(void);

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    const AP_INDIInfo& get_indi_info(void) const
    {
        return _indi_info;
    } 

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &tau(void) { return gains.tau; }

    void convert_pid();

private:
    const AP_FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    AP_Int16 _max_rate_neg;
    AP_Float _roll_ff;
    float _last_out;
    AC_PID rate_pid{0.04, 0.15, 0, 0.345, 0.666, 3, 0, 12, 150, 1};
    AP_INDI rate_indi{3, 0, 12, 2};
    float angle_err_deg;
    float ff_scale = 1.0;

    float deflection;

    AP_PIDInfo _pid_info;
    AP_INDIInfo _indi_info;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);
    float _get_rate_out_INDI(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode,
                             Matrix3f &indi_gains, AP_Plane_Shape &plane_shape, INDI_KF_Params &indi_kf_params);
    float _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
};
