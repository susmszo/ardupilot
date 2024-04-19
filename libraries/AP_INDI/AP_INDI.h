#pragma once

/// @file	AP_INDI.h
/// @brief	Plane INDI rate control and NDI attitude control algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>
#include <Filter/NotchFilter.h>
#include <Filter/AP_Filter.h>

#include "AP_INDIInfo.h"

/// @class	AP_INDI
/// @brief	Plane INDI rate control and NDI attitude control class

class AP_INDI {

public:

    struct Defaults{
        float roll_INDI_k;
        float pitch_INDI_k;
        float yaw_INDI_k;
        float roll_NDI_k;
        float pitch_NDI_k;
        float yaw_NDI_k;
        float roll_KF_Q;
        float roll_KF_R;
        float pitch_KF_Q;
        float pitch_KF_R;
        float yaw_KF_Q;
        float yaw_KF_R;
        float roll_filt_T_hz;
        float roll_filt_E_hz;
        float roll_filt_D_hz;
        float pitch_filt_T_hz;
        float pitch_filt_E_hz;
        float pitch_filt_D_hz;
        float yaw_filt_T_hz;
        float yaw_filt_E_hz;
        float yaw_filt_D_hz;
    };
    
    // Constructor for INDI
    AP_INDI(float initial_roll_INDI_k, float initial_pitch_INDI_k, float initial_yaw_INDI_k,
            float initial_roll_NDI_k, float initial_pitch_NDI_k, float initial_yaw_NDI_k,
            float initial_roll_KF_Q, float initial_roll_KF_R, float initial_pitch_KF_Q,
            float initial_pitch_KF_R, float initial_yaw_KF_Q, float initial_yaw_KF_R,
            float initial_roll_filt_T_hz, float initial_roll_filt_E_hz, float initial_roll_filt_D_hz, 
            float initial_pitch_filt_T_hz, float initial_pitch_filt_E_hz, float initial_pitch_filt_D_hz, 
            float initial_yaw_filt_T_hz, float initial_yaw_filt_E_hz, float initial_yaw_filt_D_hz);

    CLASS_NO_COPY(AP_INDI);

    Vector3f update_rate(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw, 
                         int32_t angle_meas_roll, int32_t angle_meas_pitch, int32_t angle_meas_yaw, float dt);
    Vector3f update_delta_inc(Vector3f rate_control, Vector3f rate_meas, float dt, float airspeed, Plane_Shape &plane_shape);
    
    // INDI internal KF
    KF_Update_Vars update_kalman_filter(Matrix3f A, Vector3f F, Vector3f C, Vector3f X, float Y, 
                                        Matrix3f P, float Q, float R, float dt);
    Matrix3f get_kf_state_mat(float dt);
    Vector3f get_kf_noise_mat(float dt);
    Vector3f get_kf_output_mat();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter() {
        _flags._reset_filter = true;
    }

    void set_delta(Vector3f delta)
    {
        _indi_info.delta = delta;
    }

    // get accessors
    AP_Float &roll_INDI_k() { return _roll_INDI_k; };
    AP_Float &pitch_INDI_k() { return _pitch_INDI_k; };
    AP_Float &yaw_INDI_k() { return _yaw_INDI_k; };
    AP_Float &roll_NDI_k() { return _roll_NDI_k; };
    AP_Float &pitch_NDI_k() { return _pitch_NDI_k; };
    AP_Float &yaw_NDI_k() { return _yaw_NDI_k; };
    AP_Float &roll_KF_Q() { return _roll_KF_Q; };
    AP_Float &roll_KF_R() { return _roll_KF_R; };
    AP_Float &pitch_KF_Q() { return _pitch_KF_Q; };
    AP_Float &pitch_KF_R() { return _pitch_KF_R; };
    AP_Float &yaw_KF_Q() { return _yaw_KF_Q; };
    AP_Float &yaw_KF_R() { return _yaw_KF_R; };
    AP_Float &roll_filt_T_hz() { return _roll_filt_T_hz; };
    AP_Float &roll_filt_E_hz() { return _roll_filt_E_hz; };
    AP_Float &roll_filt_D_hz() { return _roll_filt_D_hz; };
    AP_Float &pitch_filt_T_hz() { return _pitch_filt_T_hz; };
    AP_Float &pitch_filt_E_hz() { return _pitch_filt_E_hz; };
    AP_Float &pitch_filt_D_hz() { return _pitch_filt_D_hz; };
    AP_Float &yaw_filt_T_hz() { return _yaw_filt_T_hz; };
    AP_Float &yaw_filt_E_hz() { return _yaw_filt_E_hz; };
    AP_Float &yaw_filt_D_hz() { return _yaw_filt_D_hz; };

    float get_filt_T_alpha(float dt, float filt_T) const;
    float get_filt_E_alpha(float dt, float filt_E) const;
    float get_filt_D_alpha(float dt, float filt_D) const;

    // set accessors
    void roll_INDI_k(const float v) { _roll_INDI_k.set(v); };
    void pitch_INDI_k(const float v) { _pitch_INDI_k.set(v); };
    void yaw_INDI_k(const float v) { _yaw_INDI_k.set(v); };
    void roll_NDI_k(const float v) { _roll_NDI_k.set(v); };
    void pitch_NDI_k(const float v) { _pitch_NDI_k.set(v); };
    void yaw_NDI_k(const float v) { _yaw_NDI_k.set(v); };
    void roll_KF_Q(const float v) { _roll_KF_Q.set(v); };
    void roll_KF_R(const float v) { _roll_KF_R.set(v); };
    void pitch_KF_Q(const float v) { _pitch_KF_Q.set(v); };
    void pitch_KF_R(const float v) { _pitch_KF_R.set(v); };
    void yaw_KF_Q(const float v) { _yaw_KF_Q.set(v); };
    void yaw_KF_R(const float v) { _yaw_KF_R.set(v); };
    void roll_filt_T_hz(const float v) { _roll_filt_T_hz.set(fabsf(v)); };
    void roll_filt_E_hz(const float v) { _roll_filt_E_hz.set(fabsf(v)); };
    void roll_filt_D_hz(const float v) { _roll_filt_D_hz.set(fabsf(v)); };
    void pitch_filt_T_hz(const float v) { _pitch_filt_T_hz.set(fabsf(v)); };
    void pitch_filt_E_hz(const float v) { _pitch_filt_E_hz.set(fabsf(v)); };
    void pitch_filt_D_hz(const float v) { _pitch_filt_D_hz.set(fabsf(v)); };
    void yaw_filt_T_hz(const float v) { _yaw_filt_T_hz.set(fabsf(v)); };
    void yaw_filt_E_hz(const float v) { _yaw_filt_E_hz.set(fabsf(v)); };
    void yaw_filt_D_hz(const float v) { _yaw_filt_D_hz.set(fabsf(v)); };

    const AP_INDIInfo& get_indi_info(void) const { return _indi_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Float _roll_INDI_k;
    AP_Float _pitch_INDI_k;
    AP_Float _yaw_INDI_k;
    AP_Float _roll_NDI_k;
    AP_Float _pitch_NDI_k;
    AP_Float _yaw_NDI_k;
    AP_Float _roll_KF_Q;
    AP_Float _roll_KF_R;
    AP_Float _pitch_KF_Q;
    AP_Float _pitch_KF_R;
    AP_Float _yaw_KF_Q;
    AP_Float _yaw_KF_R;
    AP_Float _roll_filt_T_hz;
    AP_Float _roll_filt_E_hz;
    AP_Float _roll_filt_D_hz;
    AP_Float _pitch_filt_T_hz;
    AP_Float _pitch_filt_E_hz;
    AP_Float _pitch_filt_D_hz;
    AP_Float _yaw_filt_T_hz;
    AP_Float _yaw_filt_E_hz;
    AP_Float _yaw_filt_D_hz;

    Matrix3f _K_NDI;
    Matrix3f _K_INDI;
    Matrix3f _Mc_delta;
    Matrix3f _I;
    
    // flags
    struct ap_indi_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
        bool _inverse_N :1; // true when M_c_delta can't be inversed
    } _flags;

    // internal variables
    Vector3f _angle_target_deg;
    Vector3f _angle_meas_deg;
    Vector3f _angle_error_deg;
    Vector3f _angle_target_deg_derivative;

    Vector3f _rate_target;
    Vector3f _rate_meas;
    Vector3f _rate_error;
    Vector3f _rate_target_derivative;
    Vector3f _rate_meas_derivative; // omega_dot
    Vector3f _rate_meas_derivative_direct;

    Vector3f _delta_inc;

    Matrix3f _kf_state_mat;
    Vector3f _kf_noise_mat;
    Vector3f _kf_output_mat;
    KF_Update_Vars _kf_update_vars_R;
    KF_Update_Vars _kf_update_vars_P;
    KF_Update_Vars _kf_update_vars_Y;

    float _I_x;
    float _I_y;
    float _I_z;
    float _I_xz;
    float _C_l_a;
    float _C_l_r;
    float _C_m_e;
    float _C_n_a;
    float _C_n_r;
    float _S;
    float _b;
    float _c;
    Matrix3f _char_length_mat;

    Matrix3f _identity;

    AP_INDIInfo _indi_info;

private:

    const float default_roll_INDI_k;
    const float default_pitch_INDI_k;
    const float default_yaw_INDI_k;
    const float default_roll_NDI_k;
    const float default_pitch_NDI_k;
    const float default_yaw_NDI_k;
    const float default_roll_KF_Q;
    const float default_roll_KF_R;
    const float default_pitch_KF_Q;
    const float default_pitch_KF_R;
    const float default_yaw_KF_Q;
    const float default_yaw_KF_R;
    const float default_roll_filt_T_hz;
    const float default_roll_filt_E_hz;
    const float default_roll_filt_D_hz;
    const float default_pitch_filt_T_hz;
    const float default_pitch_filt_E_hz;
    const float default_pitch_filt_D_hz;
    const float default_yaw_filt_T_hz;
    const float default_yaw_filt_E_hz;
    const float default_yaw_filt_D_hz;

};
