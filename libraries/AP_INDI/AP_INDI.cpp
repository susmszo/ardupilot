/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include "AP_INDI.h"

const AP_Param::GroupInfo AP_INDI::var_info[] = {

    // @Param: RLL_RATE_K
    // @DisplayName: INDI roll controller gain
    // @Description: INDI roll controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_RATE_K", 0, AP_INDI, _roll_INDI_k, default_roll_INDI_k),

    // @Param: PTCH_RATE_K
    // @DisplayName: INDI pitch controller gain
    // @Description: INDI pitch controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_RATE_K", 1, AP_INDI, _pitch_INDI_k, default_pitch_INDI_k),

    // @Param: YAW_RATE_K
    // @DisplayName: INDI yaw controller gain
    // @Description: INDI yaw controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_RATE_K", 2, AP_INDI, _yaw_INDI_k, default_yaw_INDI_k),

    // @Param: RLL_ATT_K
    // @DisplayName: NDI roll controller gain
    // @Description: NDI roll controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_ATT_K", 3, AP_INDI, _roll_NDI_k, default_roll_NDI_k),

    // @Param: PTCH_ATT_K
    // @DisplayName: NDI pitch controller gain
    // @Description: NDI pitch controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_ATT_K", 4, AP_INDI, _pitch_NDI_k, default_pitch_NDI_k),

    // @Param: YAW_ATT_K
    // @DisplayName: NDI yaw controller gain
    // @Description: NDI yaw controller gain
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_ATT_K", 5, AP_INDI, _yaw_NDI_k, default_yaw_NDI_k),

    // @Param: RLL_KF_Q
    // @DisplayName: INDI roll KF process noise Cov
    // @Description: INDI roll KF process noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_KF_Q", 6, AP_INDI, _roll_KF_Q, default_roll_KF_Q),

    // @Param: RLL_KF_R
    // @DisplayName: INDI roll KF measurement noise Cov
    // @Description: INDI roll KF measurement noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_KF_R", 7, AP_INDI, _roll_KF_R, default_roll_KF_R),

    // @Param: PTCH_KF_Q
    // @DisplayName: INDI pitch KF process noise Cov
    // @Description: INDI pitch KF process noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_KF_Q", 8, AP_INDI, _pitch_KF_Q, default_pitch_KF_Q),

    // @Param: PTCH_KF_R
    // @DisplayName: INDI pitch KF measurement noise Cov
    // @Description: INDI pitch KF measurement noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_KF_R", 9, AP_INDI, _pitch_KF_R, default_pitch_KF_R),

    // @Param: YAW_KF_Q
    // @DisplayName: INDI yaw KF process noise Cov
    // @Description: INDI yaw KF process noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_KF_Q", 10, AP_INDI, _yaw_KF_Q, default_yaw_KF_Q),

    // @Param: YAW_KF_R
    // @DisplayName: INDI yaw KF measurement noise Cov
    // @Description: INDI yaw KF measurement noise Cov
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_KF_R", 11, AP_INDI, _yaw_KF_R, default_yaw_KF_R),

    // @Param: RLL_FLTT
    // @DisplayName: INDI roll Target filter frequency in Hz
    // @Description: INDI roll Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_FLTT", 12, AP_INDI, _roll_filt_T_hz, default_roll_filt_T_hz),

    // @Param: RLL_FLTE
    // @DisplayName: INDI roll Error filter frequency in Hz
    // @Description: INDI roll Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_FLTE", 13, AP_INDI, _roll_filt_E_hz, default_roll_filt_E_hz),

    // @Param: RLL_FLTD
    // @DisplayName: INDI roll Derivative filter frequency in Hz
    // @Description: INDI roll Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_FLTD", 14, AP_INDI, _roll_filt_D_hz, default_roll_filt_D_hz),

    // @Param: PTCH_FLTT
    // @DisplayName: INDI pitch Target filter frequency in Hz
    // @Description: INDI pitch Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_FLTT", 15, AP_INDI, _pitch_filt_T_hz, default_pitch_filt_T_hz),

    // @Param: PTCH_FLTE
    // @DisplayName: INDI pitch Error filter frequency in Hz
    // @Description: INDI pitch Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_FLTE", 16, AP_INDI, _pitch_filt_E_hz, default_pitch_filt_E_hz),

    // @Param: PTCH_FLTD
    // @DisplayName: INDI pitch Derivative filter frequency in Hz
    // @Description: INDI pitch Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_FLTD", 17, AP_INDI, _pitch_filt_D_hz, default_pitch_filt_D_hz),

    // @Param: YAW_FLTT
    // @DisplayName: INDI yaw Target filter frequency in Hz
    // @Description: INDI yaw Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_FLTT", 18, AP_INDI, _yaw_filt_T_hz, default_yaw_filt_T_hz),

    // @Param: YAW_FLTE
    // @DisplayName: INDI yaw Error filter frequency in Hz
    // @Description: INDI yaw Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_FLTE", 19, AP_INDI, _yaw_filt_E_hz, default_yaw_filt_E_hz),

    // @Param: YAW_FLTD
    // @DisplayName: INDI yaw Derivative filter frequency in Hz
    // @Description: INDI yaw Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_FLTD", 20, AP_INDI, _yaw_filt_D_hz, default_yaw_filt_D_hz),

    // @Param: RLL_DELIM
    // @DisplayName: INDI aileron deflection limit in deg
    // @Description: INDI aileron deflection limit in deg
    // @Units: deg
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("RLL_DELIM", 21, AP_INDI, _roll_delta_limit_deg, default_roll_delta_limit_deg),

    // @Param: PTCH_DELIM
    // @DisplayName: INDI elevator deflection limit in deg
    // @Description: INDI elevator deflection limit in deg
    // @Units: deg
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("PTCH_DELIM", 22, AP_INDI, _pitch_delta_limit_deg, default_pitch_delta_limit_deg),

    // @Param: YAW_DELIM
    // @DisplayName: INDI rudder deflection limit in deg
    // @Description: INDI rudder deflection limit in deg
    // @Units: deg
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("YAW_DELIM", 23, AP_INDI, _yaw_delta_limit_deg, default_yaw_delta_limit_deg),

    AP_GROUPEND
};

// Constructor
AP_INDI::AP_INDI(float initial_roll_INDI_k, float initial_pitch_INDI_k, float initial_yaw_INDI_k,
                 float initial_roll_NDI_k, float initial_pitch_NDI_k, float initial_yaw_NDI_k,
                 float initial_roll_KF_Q, float initial_roll_KF_R, float initial_pitch_KF_Q,
                 float initial_pitch_KF_R, float initial_yaw_KF_Q, float initial_yaw_KF_R,
                 float initial_roll_filt_T_hz, float initial_roll_filt_E_hz, float initial_roll_filt_D_hz, 
                 float initial_pitch_filt_T_hz, float initial_pitch_filt_E_hz, float initial_pitch_filt_D_hz, 
                 float initial_yaw_filt_T_hz, float initial_yaw_filt_E_hz, float initial_yaw_filt_D_hz,
                 float initial_roll_delta_limit_deg, float initial_pitch_delta_limit_deg, float initial_yaw_delta_limit_deg) :
    default_roll_INDI_k(initial_roll_INDI_k),
    default_pitch_INDI_k(initial_pitch_INDI_k),
    default_yaw_INDI_k(initial_yaw_INDI_k),
    default_roll_NDI_k(initial_roll_NDI_k),
    default_pitch_NDI_k(initial_pitch_NDI_k),
    default_yaw_NDI_k(initial_yaw_NDI_k),
    default_roll_KF_Q(initial_roll_KF_Q),
    default_roll_KF_R(initial_roll_KF_R),
    default_pitch_KF_Q(initial_pitch_KF_Q),
    default_pitch_KF_R(initial_pitch_KF_R),
    default_yaw_KF_Q(initial_yaw_KF_Q),
    default_yaw_KF_R(initial_yaw_KF_R),
    default_roll_filt_T_hz(initial_roll_filt_T_hz),
    default_roll_filt_E_hz(initial_roll_filt_E_hz),
    default_roll_filt_D_hz(initial_roll_filt_D_hz),
    default_pitch_filt_T_hz(initial_pitch_filt_T_hz),
    default_pitch_filt_E_hz(initial_pitch_filt_E_hz),
    default_pitch_filt_D_hz(initial_pitch_filt_D_hz),
    default_yaw_filt_T_hz(initial_yaw_filt_T_hz),
    default_yaw_filt_E_hz(initial_yaw_filt_E_hz),
    default_yaw_filt_D_hz(initial_yaw_filt_D_hz),
    default_roll_delta_limit_deg(initial_roll_delta_limit_deg),
    default_pitch_delta_limit_deg(initial_pitch_delta_limit_deg),
    default_yaw_delta_limit_deg(initial_yaw_delta_limit_deg)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // reset input filter to first value received
    _flags._reset_filter = true;
    _flags._inverse_N = false;
    _flags._reset_NDI = true;

    _identity.identity();

    // memset(&_indi_info, 0, sizeof(_indi_info));
}

//  update_rate - set target and measured inputs to NDI controller and calculate outputs
//  target and measurement are int32_t in centidegrees
//  no filter
Vector3f AP_INDI::update_rate(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw, 
                              int32_t angle_meas_roll, int32_t angle_meas_pitch, int32_t angle_meas_yaw, float dt)
{
    Vector3f angle_target_deg = Vector3f(angle_target_roll * 0.01, angle_target_pitch * 0.01, angle_target_yaw * 0.01);
    Vector3f angle_meas_deg = Vector3f(angle_meas_roll * 0.01, angle_meas_pitch * 0.01, angle_meas_yaw * 0.01);

    // _angle_error_deg.z = 0;

    // don't process inf or NaN
    if (!isfinite(_angle_error_deg.x) || !isfinite(_angle_error_deg.y) || !isfinite(_angle_error_deg.z)) {
        return Vector3f(0, 0, 0);
    }

    // reset input filter to value received and re-initialization
    _indi_info.reset_NDI = _flags._reset_NDI;
    if (_flags._reset_NDI) {
        _flags._reset_NDI = false;
        _angle_target_deg = angle_target_deg;
        _angle_error_deg = _angle_target_deg - angle_meas_deg;
        _angle_target_deg_derivative = Vector3f(0, 0, 0);
    } else {
        Vector3f angle_target_deg_last = _angle_target_deg;
        Vector3f angle_error_deg = _angle_target_deg - angle_meas_deg;
        _angle_target_deg.x += (angle_target_deg.x - _angle_target_deg.x) * get_filt_T_alpha(dt, _roll_filt_T_hz);
        _angle_target_deg.y += (angle_target_deg.y - _angle_target_deg.y) * get_filt_T_alpha(dt, _pitch_filt_T_hz);
        _angle_target_deg.z += (angle_target_deg.z - _angle_target_deg.z) * get_filt_T_alpha(dt, _yaw_filt_T_hz);
        _angle_error_deg.x += (angle_error_deg.x - _angle_error_deg.x) * get_filt_E_alpha(dt, _roll_filt_E_hz);
        _angle_error_deg.y += (angle_error_deg.y - _angle_error_deg.y) * get_filt_E_alpha(dt, _pitch_filt_E_hz);
        _angle_error_deg.z += (angle_error_deg.z - _angle_error_deg.z) * get_filt_E_alpha(dt, _yaw_filt_E_hz);
        // _angle_target_deg.x = angle_target_deg.x; 
        // _angle_target_deg.y = angle_target_deg.y; 
        // _angle_target_deg.z = angle_target_deg.z; 
        // _angle_error_deg.x = angle_target_deg.x - angle_meas_deg.x;
        // _angle_error_deg.y = angle_target_deg.y - angle_meas_deg.y;
        // _angle_error_deg.z = angle_target_deg.z - angle_meas_deg.z;
        _angle_meas_deg = angle_meas_deg;

        // calculate derivative
        if (is_positive(dt)) {
            _angle_target_deg_derivative = (_angle_target_deg - angle_target_deg_last) / dt;
        }
    }

    _K_NDI = Matrix3f(_roll_NDI_k, 0, 0, 0, _pitch_NDI_k, 0, 0, 0, _yaw_NDI_k);

    float phi = radians(_angle_meas_deg.x);
    float theta = radians(_angle_meas_deg.y);
    Matrix3f transform_mat(1, 0, -sinf(theta), 0, cosf(phi), (cosf(theta) * sinf(phi)), 0, -sinf(phi), (cosf(theta) * cosf(phi)));

    Vector3f v = _K_NDI * (_angle_error_deg * DEG_TO_RAD);
    _indi_info.ndi_v = v;

    v += _angle_target_deg_derivative * DEG_TO_RAD;
    _indi_info.ndi_v_ = v;

    // rate_c in rad/s
    // Vector3f rate_control = transform_mat * (_K_NDI * (_angle_error_deg * DEG_TO_RAD) + _angle_target_deg_derivative * DEG_TO_RAD);
    Vector3f rate_control = transform_mat * v;

    _angle_target_deg = angle_target_deg;

    _indi_info.angle_target = angle_target_deg;
    _indi_info.angle_actual = _angle_meas_deg;
    _indi_info.angle_error = _angle_error_deg;
    _indi_info.angle_target_derivative = _angle_target_deg_derivative;
    _indi_info.rate_control = rate_control;
    
    return rate_control;
}

//  update_delta_inc - set target and measured inputs to INDI controller and calculate outputs
//  target and measurement are vectors in rad/s
//  target and error are filtered
Vector3f AP_INDI::update_delta_inc(Vector3f rate_target, Vector3f rate_meas, float dt, float airspeed, Plane_Shape &plane_shape)
{
    // don't process inf or NaN
    if (!isfinite(rate_target.x) || !isfinite(rate_target.y) || !isfinite(rate_target.z)) {
        return Vector3f(0, 0, 0);
    }

    // update plane shape parameters
    _I_x = plane_shape.I_x;
    _I_y = plane_shape.I_x;
    _I_z = plane_shape.I_x;
    _I_xz = plane_shape.I_xz;

    // reset input filter to value received and re-initialization
    _indi_info.reset = _flags._reset_filter;
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _rate_target = rate_target;
        _rate_error = _rate_target - rate_meas;
        _rate_target_derivative = Vector3f(0, 0, 0);
        _rate_meas_derivative_direct = Vector3f(0, 0, 0);
        // kalman filter initialization
        _kf_state_mat = get_kf_state_mat(dt);
        _kf_noise_mat = get_kf_noise_mat(dt);
        _kf_output_mat = get_kf_output_mat();
        _kf_update_vars_R.X_hat = Vector3f(rate_meas.x, 0.0f, 0.0f);
        _kf_update_vars_P.X_hat = Vector3f(rate_meas.y, 0.0f, 0.0f);
        _kf_update_vars_Y.X_hat = Vector3f(rate_meas.z, 0.0f, 0.0f);
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
        _char_length_mat = Matrix3f(_b, 0, 0, 0, _c, 0, 0, 0, _b);
    } else {
        Vector3f rate_target_last = _rate_target;
        Vector3f rate_meas_last = _rate_meas;
        Vector3f rate_error = rate_target - rate_meas;
        _rate_target.x += (rate_target.x - _rate_target.x) * get_filt_T_alpha(dt, _roll_filt_T_hz);
        _rate_target.y += (rate_target.y - _rate_target.y) * get_filt_T_alpha(dt, _pitch_filt_T_hz);
        _rate_target.z += (rate_target.z - _rate_target.z) * get_filt_T_alpha(dt, _yaw_filt_T_hz);
        _rate_error.x += (rate_error.x - _rate_error.x) * get_filt_E_alpha(dt, _roll_filt_E_hz);
        _rate_error.y += (rate_error.y - _rate_error.y) * get_filt_E_alpha(dt, _pitch_filt_E_hz);
        _rate_error.z += (rate_error.z - _rate_error.z) * get_filt_E_alpha(dt, _yaw_filt_E_hz);
        // _rate_target.x = rate_target.x; 
        // _rate_target.y = rate_target.y; 
        // _rate_target.z = rate_target.z; 
        // _rate_error.x = rate_target.x - rate_meas.x;
        // _rate_error.y = rate_target.y - rate_meas.y;
        // _rate_error.z = rate_target.z - rate_meas.z;
        _rate_meas = rate_meas;
        // calculate and filter derivative
        if (is_positive(dt)) {
            _rate_target_derivative = (_rate_target - rate_target_last) / dt;
            // directly derivate
            Vector3f rate_meas_derivative_direct = (rate_meas - rate_meas_last) / dt;
            // _rate_meas_derivative_direct = (rate_meas - rate_meas_last) / dt;
            _rate_meas_derivative_direct.x += (rate_meas_derivative_direct.x - _rate_meas_derivative_direct.x) * get_filt_D_alpha(dt, _roll_filt_D_hz); 
            _rate_meas_derivative_direct.y += (rate_meas_derivative_direct.y - _rate_meas_derivative_direct.y) * get_filt_D_alpha(dt, _pitch_filt_D_hz); 
            _rate_meas_derivative_direct.z += (rate_meas_derivative_direct.z - _rate_meas_derivative_direct.z) * get_filt_D_alpha(dt, _yaw_filt_D_hz); 
        }
    }

    _K_INDI = Matrix3f(_roll_INDI_k, 0, 0, 0, _pitch_INDI_k, 0, 0, 0, _yaw_INDI_k);
    
    _Mc_delta = _char_length_mat * Matrix3f(_C_l_a, 0, _C_l_r, 0, _C_m_e, 0, _C_n_a, 0, _C_n_r) * 0.5 * SSL_AIR_DENSITY * powf(airspeed, 2) * _S;
    _I = Matrix3f(_I_x, 0, -_I_xz, 0, _I_y, 0, -_I_xz, 0, _I_z);

    // virtual control input, K*e
    Vector3f v = _K_INDI * _rate_error;
    _indi_info.v = v;

    // kf
    _kf_update_vars_R = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_R.X_hat, rate_meas.x,
                                             _kf_update_vars_R.P_cov, _roll_KF_Q, _roll_KF_R, dt);
    _kf_update_vars_P = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_P.X_hat, rate_meas.y,
                                             _kf_update_vars_P.P_cov, _pitch_KF_Q, _pitch_KF_R, dt);
    _kf_update_vars_Y = update_kalman_filter(_kf_state_mat, _kf_noise_mat, _kf_output_mat, _kf_update_vars_Y.X_hat, rate_meas.z,
                                             _kf_update_vars_Y.P_cov, _yaw_KF_Q, _yaw_KF_R, dt);
    _rate_meas_derivative.x = _kf_update_vars_R.X_hat.y;
    _rate_meas_derivative.y = _kf_update_vars_P.X_hat.y;
    _rate_meas_derivative.z = _kf_update_vars_Y.X_hat.y;

    // v += (_rate_target_derivative - _rate_meas_derivative);
    v += (_rate_target_derivative - _rate_meas_derivative_direct);
    _indi_info.v_ = v;

    // _delta_inc in rad
    if (_Mc_delta.invert()) {
        _delta_inc = _Mc_delta * _I * v;
        _flags._inverse_N = false;
    } else {
        _delta_inc = Vector3f(0, 0, 0);
        _flags._inverse_N = true;
    }
    _indi_info.inverse_N = _flags._inverse_N;

    _indi_info.delta_inc = _delta_inc * RAD_TO_DEG;

    // output limit
    if (is_positive(_roll_delta_limit_deg)) {
        _delta_inc.x = constrain_float(_delta_inc.x, -_roll_delta_limit_deg * DEG_TO_RAD, _roll_delta_limit_deg * DEG_TO_RAD);
    }
    if (is_positive(_pitch_delta_limit_deg)) {
        _delta_inc.y = constrain_float(_delta_inc.y, -_pitch_delta_limit_deg * DEG_TO_RAD, _pitch_delta_limit_deg * DEG_TO_RAD);
    }
    if (is_positive(_yaw_delta_limit_deg)) {
        _delta_inc.z = constrain_float(_delta_inc.z, -_yaw_delta_limit_deg * DEG_TO_RAD, _yaw_delta_limit_deg * DEG_TO_RAD);
    }
    
    _indi_info.delta_inc_limit = _delta_inc * RAD_TO_DEG;

    _indi_info.rate_target = _rate_target;
    _indi_info.rate_actual = _rate_meas;
    _indi_info.rate_error = _rate_error;
    _indi_info.rate_target_derivative = _rate_target_derivative;
    _indi_info.rate_meas_derivative_direct = _rate_meas_derivative_direct;
    _indi_info.roll_kf_vars = _kf_update_vars_R;
    _indi_info.pitch_kf_vars = _kf_update_vars_P;
    _indi_info.yaw_kf_vars = _kf_update_vars_Y;

    return _delta_inc;
}

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
    Vector3f noise_mat(powf((dt / 2), 2), (dt / 2), 1);
    return noise_mat;
}

Vector3f AP_INDI::get_kf_output_mat()
{
    Vector3f output_mat(1, 0, 0);
    return output_mat;
}

// get_filt_T_alpha - get the target filter alpha
float AP_INDI::get_filt_T_alpha(float dt, float filt_T) const
{
    return calc_lowpass_alpha_dt(dt, filt_T);
}

// get_filt_E_alpha - get the error filter alpha
float AP_INDI::get_filt_E_alpha(float dt, float filt_E) const
{
    return calc_lowpass_alpha_dt(dt, filt_E);
}

// get_filt_D_alpha - get the derivative filter alpha
float AP_INDI::get_filt_D_alpha(float dt, float filt_D) const
{
    return calc_lowpass_alpha_dt(dt, filt_D);
}
