#pragma once

struct KF_Update_Vars {
    Matrix3f P_cov; 
    Vector3f X_hat;
};

struct AP_INDIInfo {
    Vector3f angle_target;
    Vector3f angle_actual;
    Vector3f angle_error; 
    Vector3f angle_target_derivative;
    Vector3f ndi_v;
    Vector3f ndi_v_;
    Vector3f rate_control;

    Vector3f rate_target;
    Vector3f rate_actual;
    Vector3f rate_error; 
    Vector3f rate_target_derivative;
    Vector3f rate_meas_derivative_direct;
    Vector3f v;
    Vector3f v_;
    Vector3f delta_inc;
    Vector3f delta_inc_limit;
    Vector3f delta;
    Vector3f delta_target;

    Vector3f rate_I;

    KF_Update_Vars roll_kf_vars;
    KF_Update_Vars pitch_kf_vars;
    KF_Update_Vars yaw_kf_vars;
    
    bool inverse_N;
    bool reset;
    bool reset_NDI;
};

struct Plane_Shape {
    float I_x;
    float I_y;
    float I_z;
    float I_xz;
    float C_l_a;
    float C_l_r;
    float C_m_e;
    float C_n_a;
    float C_n_r;
    float S;
    float b;
    float c;
};
