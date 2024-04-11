#pragma once

struct AP_INDIInfo {
    int index;

    Vector3f target;
    Vector3f actual;
    Vector3f error;
    float delta_inc;
    KF_Update_Vars kf_update_vars;
    
    bool reset;
};

struct KF_Update_Vars {
    Matrix3f P_cov; 
    Vector3f X_hat;
};

struct AP_Plane_Shape {
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
}Plane_Shape;

struct INDI_KF_Params {
    float Q_roll;
    float R_roll;
    float Q_pitch;
    float R_pitch;
    float Q_yaw;
    float R_yaw;
};
