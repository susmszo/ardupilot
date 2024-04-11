#pragma once

/// @file	AP_INDI.h
/// @brief	Plane INDI rate control algorithm, with EEPROM-backed storage of constants.

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
/// @brief	Plane INDI rate control class

// class Plane_Shape {

// public:

//     struct Defaults {
//         float I_x;
//         float I_y;
//         float I_z;
//         float I_xz;
//         float C_l_a;
//         float C_l_r;
//         float C_m_e;
//         float C_n_a;
//         float C_n_r;
//         float S;
//         float b;
//         float c;
//     };

//     Plane_Shape(float initial_I_x, float initial_I_y, float initial_I_z, float initial_I_xz,
//                 float initial_C_l_a, float initial_C_l_r, float initial_C_m_e, float initial_C_n_a,
//                 float initial_C_n_r, float initial_S, float initial_b, float initial_c);
    
// };

// Plane_Shape::Plane_Shape(/* args */)
// {
// }

class AP_INDI {

public:

    struct Defaults{
        float filt_T_hz;
        float filt_E_hz;
        float filt_D_hz;
        int index;
    };
    
    // Constructor for INDI
    AP_INDI(float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, int initial_index);
    AP_INDI(const AP_INDI::Defaults &defaults) :
        AP_INDI(
            defaults.filt_T_hz,
            defaults.filt_E_hz,
            defaults.filt_D_hz,
            defaults.index
            )
        { }

    CLASS_NO_COPY(AP_INDI);

    float update_all(Vector3f target, Vector3f measurement, float dt, Matrix3f K, INDI_KF_Params kf_params,
                     float air_dens, float airspeed, AP_Plane_Shape plane_shape, const AP_AHRS &_ahrs);
    // float update_error(float error, float dt, float air_dens, float airspeed, int index, AP_Plane_Shape plane_shape);
    
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

    // get accessors
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }

    float get_filt_T_alpha(float dt) const;
    float get_filt_E_alpha(float dt) const;
    float get_filt_D_alpha(float dt) const;

    // set accessors
    void filt_T_hz(const float v);
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);

    const AP_INDIInfo& get_indi_info(void) const { return _indi_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Float _filt_T_hz;         // PID target filter frequency in Hz
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz

    // flags
    struct ap_indi_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    int _index;

    Vector3f _target;
    Vector3f _measurement;
    Vector3f _error;
    Vector3f _target_derivative;
    Vector3f _meas_derivative; // omega_dot
    Vector3f _v; // virtual control input
    float _delta;
    
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

    Matrix3f _identity;

    AP_INDIInfo _indi_info;
    AP_Plane_Shape _plane_shape;

private:

    const float default_filt_T_hz;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
    const int default_index;

};
