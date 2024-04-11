/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include "AP_INDI.h"

const AP_Param::GroupInfo AP_INDI::var_info[] = {
    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 0, AP_INDI, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 1, AP_INDI, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 2, AP_INDI, _filt_D_hz, default_filt_D_hz),

    AP_GROUPEND
};

// Constructor
AP_INDI::AP_INDI(float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, int initial_index) :
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_index(initial_index)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // reset input filter to first value received
    _flags._reset_filter = true;

    _index = default_index;

    _identity.identity();

    memset(&_indi_info, 0, sizeof(_indi_info));
}

// filt_T_hz - set target filter hz
void AP_INDI::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AP_INDI::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AP_INDI::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

//  update_all - set target and measured inputs to INDI controller and calculate outputs
//  process target and measured angular rate, return deflection increment 
//  target and measurement are vectors in rad
//  target and error are filtered
//  index: 1 for Roll, 2 for Pitch, 3 for Yaw
float AP_INDI::update_all(Vector3f target, Vector3f measurement, float dt, Matrix3f K, INDI_KF_Params kf_params,  
                          float air_dens, float airspeed, AP_Plane_Shape plane_shape, const AP_AHRS &_ahrs)
{
    // don't process inf or NaN
    if (!isfinite(target.x) || !isfinite(target.y) || !isfinite(target.z) ||
        !isfinite(measurement.x) || !isfinite(measurement.y) || !isfinite(measurement.z)) {
        return 0.0f;
    }
    
    int index = default_index;
    _indi_info.index = index;

    // process plane shape parameters
    _I_x = plane_shape.I_x;
    _I_y = plane_shape.I_x;
    _I_z = plane_shape.I_x;
    _I_xz = plane_shape.I_xz;

    // reset input filter to value received and re-initialization
    _indi_info.reset = _flags._reset_filter;
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _target_derivative.x = 0.0f;
        _target_derivative.y = 0.0f;
        _target_derivative.z = 0.0f;
        // kalman filter initialization
        _kf_state_mat = get_kf_state_mat(dt);
        _kf_noise_mat = get_kf_noise_mat(dt);
        _kf_output_mat = get_kf_output_mat();
        _kf_update_vars_R.X_hat = Vector3f(_ahrs.get_gyro().x, 0.0f, 0.0f);
        _kf_update_vars_P.X_hat = Vector3f(_ahrs.get_gyro().y, 0.0f, 0.0f);
        _kf_update_vars_Y.X_hat = Vector3f(_ahrs.get_gyro().z, 0.0f, 0.0f);
        _kf_update_vars_R.P_cov = _identity;
        _kf_update_vars_P.P_cov = _identity;
        _kf_update_vars_Y.P_cov = _identity;
        // plane shape initialization
        _C_l_a = plane_shape.C_l_a;
        _C_l_r = plane_shape.C_l_r;
        _C_m_e = plane_shape.C_m_e;
        _C_n_a = plane_shape.C_n_a;
        _C_n_r = plane_shape.C_n_r;
        _S = plane_shape.S;
        _b = plane_shape.b;
        _c = plane_shape.c;
    } else {
        Vector3f target_last = _target;
        // Vector3f measurement_last = _measurement;
        Vector3f error = _target - measurement;
        _target += (target - _target) * get_filt_T_alpha(dt);
        _error += (error - _error) * get_filt_E_alpha(dt);
        _measurement = measurement;
        // calculate and filter derivative
        if (is_positive(dt)) {
            _target_derivative = (_target - target_last) / dt;
            // _meas_derivative = (_measurement - measurement_last) / dt;
        }
    }

    // calculate lumped gain for controller
    float lumped_gain = 1 / (0.5 * air_dens * airspeed * airspeed * _S * _C_m_e * (_C_l_a * _C_n_r - _C_l_r * _C_n_a));

    // control output in rad, increment form
    float delta_inc = 1;

    // virtual control input
    _v = K * _error;
    Vector3f v = _v;

    float Q_roll = kf_params.Q_roll;
    float R_roll = kf_params.R_roll;
    float Q_pitch = kf_params.Q_pitch;
    float R_pitch = kf_params.R_pitch;
    float Q_yaw = kf_params.Q_yaw;
    float R_yaw = kf_params.R_yaw;

    // update kalman filter upon the axis
    switch (index)
    {
    // roll
    case 1:
        _kf_update_vars_R = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_R.X_hat, _ahrs.get_gyro().x,
                                                 _kf_update_vars_R.P_cov, Q_roll, R_roll, dt);
        _kf_update_vars_Y = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_Y.X_hat, _ahrs.get_gyro().z,
                                                 _kf_update_vars_Y.P_cov, Q_yaw, R_yaw, dt);
        _meas_derivative.x = _kf_update_vars_R.X_hat.y;
        _meas_derivative.z = _kf_update_vars_Y.X_hat.y;
        v += (_target_derivative - _meas_derivative);
        delta_inc *= lumped_gain / _b * _C_m_e * ((_I_x * _C_n_r + _I_xz * _C_l_r) * v.x - (_I_xz * _C_n_r + _I_z * _C_l_r) * v.z);
        _indi_info.kf_update_vars = _kf_update_vars_R;
        break;
    // pitch
    case 2:
        _kf_update_vars_P = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_P.X_hat, _ahrs.get_gyro().y,
                                                 _kf_update_vars_P.P_cov, Q_pitch, R_pitch, dt);
        _meas_derivative.y = _kf_update_vars_P.X_hat.y;
        v += (_target_derivative - _meas_derivative);
        delta_inc *= lumped_gain / _c * (_I_xz * _C_n_r - _I_z * _C_l_r) * v.y;
        _indi_info.kf_update_vars = _kf_update_vars_P;
        break;
    // yaw
    case 3:
        _kf_update_vars_R = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_R.X_hat, _ahrs.get_gyro().x,
                                                 _kf_update_vars_R.P_cov, Q_roll, R_roll, dt);
        _kf_update_vars_Y = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_Y.X_hat, _ahrs.get_gyro().z,
                                                 _kf_update_vars_Y.P_cov, Q_yaw, R_yaw, dt);
        _meas_derivative.x = _kf_update_vars_R.X_hat.y;
        _meas_derivative.z = _kf_update_vars_Y.X_hat.y;
        v += (_target_derivative - _meas_derivative);  
        delta_inc *= lumped_gain / _b * _C_m_e * (- (_I_x * _C_n_a + _I_xz * _C_l_a) * v.x + (_I_xz * _C_n_a + _I_z * _C_l_a) * v.z);
        _indi_info.kf_update_vars = _kf_update_vars_Y;
        break;
    // don't process with wrong index
    default:
        return 0.0f;
        break;
    }

    _indi_info.target = _target;
    _indi_info.actual = measurement;
    _indi_info.error = _error;
    _indi_info.delta_inc = delta_inc;

    return delta_inc;
}

//  update_error - set error input to INDI controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
// float AP_INDI::update_error(float error, float dt, float air_dens, float airspeed, int index, AP_Plane_Shape plane_shape)
// {
//     // don't process inf or NaN
//     if (!isfinite(error)) {
//         return 0.0f;
//     }

//     // Reuse update all code path, zero target and pass negative error as measurement
//     // Passing as measurement bypasses any target filtering to maintain behaviour
//     // Negate as update all calculates error as target - measurement
//     _target = 0.0;
//     const float output = update_all(0.0, -error, air_dens, dt, airspeed, index, plane_shape);

//     return output;
// }

// INDI internal KF
KF_Update_Vars AP_INDI::update_kalman_filter(Matrix3f A, Vector3f F, Vector3f C, Vector3f X, float Y, 
                                             Matrix3f P, float Q, float R, float dt)
{
    Matrix3f P_hat = A * P * A.transposed() + (F * Q).mul_rowcol(F);
    Vector3f X_hat = A * X;
    Vector3f kf_gain = P_hat * C / (C.row_times_mat(P_hat) * C + R);
    P_hat = (_identity - kf_gain.mul_rowcol(C)) * P_hat;
    X_hat = X_hat + kf_gain * (Y - C * X_hat);
    KF_Update_Vars kf_update_vars;
    kf_update_vars.P_cov = P_hat;
    kf_update_vars.X_hat = X_hat;
    return kf_update_vars;
}

Matrix3f AP_INDI::get_kf_state_mat(float dt)
{
    Matrix3f state_mat(1, dt, powf((dt / 2), 2), 0, 1, (dt / 2), 0, 0, 1);
    return state_mat;
}

Vector3f AP_INDI::get_kf_noise_mat(float dt)
{
    Vector3f control_mat(powf((dt / 2), 2), (dt / 2), 1);
    return control_mat;
}

Vector3f AP_INDI::get_kf_output_mat()
{
    Vector3f output_mat(1, 0, 0);
    return output_mat;
}

// get_filt_T_alpha - get the target filter alpha
float AP_INDI::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AP_INDI::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AP_INDI::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}
