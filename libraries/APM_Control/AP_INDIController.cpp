/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_INDIController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_INDIController::var_info[] = {

    // @Param: RLL_RMAX
    // @DisplayName: INDI Maximum Roll Rate
    // @Description: This sets the maximum roll rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables this limit.
    // @Range: 0 180
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RLL_RMAX", 1, AP_INDIController, gains_roll.rmax_pos, 0.0f),

    // @Param: PTCH_RMAX_P
    // @DisplayName: INDI Pitch up max rate
    // @Description: This sets the maximum nose up pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PTCH_RMAX_P", 2, AP_INDIController, gains_pitch.rmax_pos, 0.0f),

    // @Param: PTCH_RMAX_N
    // @DisplayName: INDI Pitch down max rate
    // @Description: This sets the maximum nose down pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PTCH_RMAX_N", 3, AP_INDIController, gains_pitch.rmax_neg, 0.0f),

    // @Param: PTCH_RLL_FF
    // @DisplayName: INDI Roll compensation
    // @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
    // @Range: 0.7 1.5
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("PTCH_RLL_FF", 4, AP_INDIController, _roll_ff_to_pitch, 1.0f),

    // @Param: YAW_RLL_FF
    // @DisplayName: INDI Yaw coordination gain
    // @Description: Gain to the yaw rate required to keep it consistent with the turn rate in a coordinated turn. Corrects for yaw tendencies after the turn is established. Increase yaw into the turn by raising. Increase yaw out of the turn by decreasing. Values outside of 0.9-1.1 range indicate airspeed calibration problems.
    // @Range: 0.8 1.2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("YAW_RLL_FF", 5, AP_INDIController, _yaw_ff, 1),

    // @Param: RLL_RATE_K
    // @DisplayName: INDI roll controller gain
    // @Description: INDI roll controller gain
    // @User: Standard

    // @Param: PTCH_RATE_K
    // @DisplayName: INDI pitch controller gain
    // @Description: INDI pitch controller gain
    // @User: Standard

    // @Param: YAW_RATE_K
    // @DisplayName: INDI yaw controller gain
    // @Description: INDI yaw controller gain
    // @User: Standard

    // @Param: RLL_ATT_K
    // @DisplayName: NDI roll controller gain
    // @Description: NDI roll controller gain
    // @User: Standard

    // @Param: PTCH_ATT_K
    // @DisplayName: NDI pitch controller gain
    // @Description: NDI pitch controller gain
    // @User: Standard

    // @Param: YAW_ATT_K
    // @DisplayName: NDI yaw controller gain
    // @Description: NDI yaw controller gain
    // @User: Standard

    // @Param: RLL_KF_Q
    // @DisplayName: INDI roll KF process noise Cov
    // @Description: INDI roll KF process noise Cov
    // @User: Standard

    // @Param: RLL_KF_R
    // @DisplayName: INDI roll KF measurement noise Cov
    // @Description: INDI roll KF measurement noise Cov
    // @User: Standard

    // @Param: PTCH_KF_Q
    // @DisplayName: INDI pitch KF process noise Cov
    // @Description: INDI pitch KF process noise Cov
    // @User: Standard

    // @Param: PTCH_KF_R
    // @DisplayName: INDI pitch KF measurement noise Cov
    // @Description: INDI pitch KF measurement noise Cov
    // @User: Standard

    // @Param: YAW_KF_Q
    // @DisplayName: INDI yaw KF process noise Cov
    // @Description: INDI yaw KF process noise Cov
    // @User: Standard

    // @Param: YAW_KF_R
    // @DisplayName: INDI yaw KF measurement noise Cov
    // @Description: INDI yaw KF measurement noise Cov
    // @User: Standard

    // @Param: RLL_FLTT
    // @DisplayName: INDI roll Target filter frequency in Hz
    // @Description: INDI roll Target filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RLL_FLTE
    // @DisplayName: INDI roll Error filter frequency in Hz
    // @Description: INDI roll Error filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RLL_FLTD
    // @DisplayName: INDI roll Derivative filter frequency in Hz
    // @Description: INDI roll Derivative filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PTCH_FLTT
    // @DisplayName: INDI pitch Target filter frequency in Hz
    // @Description: INDI pitch Target filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PTCH_FLTE
    // @DisplayName: INDI pitch Error filter frequency in Hz
    // @Description: INDI pitch Error filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PTCH_FLTD
    // @DisplayName: INDI pitch Derivative filter frequency in Hz
    // @Description: INDI pitch Derivative filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: YAW_FLTT
    // @DisplayName: INDI yaw Target filter frequency in Hz
    // @Description: INDI yaw Target filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: YAW_FLTE
    // @DisplayName: INDI yaw Error filter frequency in Hz
    // @Description: INDI yaw Error filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: YAW_FLTD
    // @DisplayName: INDI yaw Derivative filter frequency in Hz
    // @Description: INDI yaw Derivative filter frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RLL_DELIM
    // @DisplayName: INDI aileron deflection limit in deg
    // @Description: INDI aileron deflection limit in deg
    // @Units: deg
    // @User: Standard

    // @Param: PTCH_DELIM
    // @DisplayName: INDI elevator deflection limit in deg
    // @Description: INDI elevator deflection limit in deg
    // @Units: deg
    // @User: Standard

    // @Param: YAW_DELIM
    // @DisplayName: INDI rudder deflection limit in deg
    // @Description: INDI rudder deflection limit in deg
    // @Units: deg
    // @User: Standard

    AP_SUBGROUPINFO(rate_indi, "", 6, AP_INDIController, AP_INDI),
 
    AP_GROUPEND
};

AP_INDIController::AP_INDIController(const AP_FixedWing &parms)
    : aparm(parms)
{
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AP_INDIController::_get_rate_out_INDI(Vector3f rate_desired, float airspeed, Plane_Shape &plane_shape)
{
    const float dt = AP::scheduler().get_loop_period_s();

    const AP_AHRS &_ahrs = AP::ahrs();
    
    Vector3f rate_meas = _ahrs.get_gyro();

    // rate_meas.z = 0;

    // in rad
    Vector3f delta_inc = rate_indi.update_delta_inc(rate_desired, rate_meas, dt, airspeed, plane_shape);

    delta_inc.x = -delta_inc.x;
    // delta_inc.x = 0;
    delta_inc.y = -delta_inc.y;
    // delta_inc.z = 0;
    delta_inc.z = -delta_inc.z;

    _indi_info = rate_indi.get_indi_info();

    return delta_inc;
}

Vector3f AP_INDIController::_get_att_out_INDI(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw)
{
    const float dt = AP::scheduler().get_loop_period_s();

    const AP_AHRS &_ahrs = AP::ahrs();

    return rate_indi.update_rate(angle_target_roll, angle_target_pitch, angle_target_yaw, _ahrs.roll_sensor, _ahrs.pitch_sensor, _ahrs.yaw_sensor, dt);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.

  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_INDIController::_get_coordination_rate_offset_pitch(float &aspeed, bool &inverted) const
{
    float rate_offset;
    float bank_angle = AP::ahrs().get_roll();

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < radians(90))	{
        bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
    } else {
        inverted = true;
        if (bank_angle > 0.0f) {
            bank_angle = constrain_float(bank_angle,radians(100),radians(180));
        } else {
            bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
        }
    }
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.airspeed_estimate(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.get_pitch())*fabsf(ToDeg((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff_to_pitch;
    }
    if (inverted) {
        rate_offset = -rate_offset;
    }
    return rate_offset;
}

Vector3f AP_INDIController::get_servo_out_INDI(int32_t angle_target_roll, int32_t angle_target_pitch, int32_t angle_target_yaw, Plane_Shape &plane_shape)
{
    // rad/s
    Vector3f rate_desired = _get_att_out_INDI(angle_target_roll, angle_target_pitch, angle_target_yaw);

    const AP_AHRS &_ahrs = AP::ahrs();
    
    float rate_offset_pitch;
    float aspeed;
    bool inverted;
    float bank_angle = _ahrs.get_roll();

    /*
    roll rate process
    */

    rate_desired.x *= RAD_TO_DEG;
    if (gains_roll.rmax_pos && rate_desired.x < -gains_roll.rmax_pos) {
        rate_desired.x = - gains_roll.rmax_pos;
    } else if (gains_roll.rmax_pos && rate_desired.x > gains_roll.rmax_pos) {
        rate_desired.x = gains_roll.rmax_pos;
    }

    rate_desired.x *= DEG_TO_RAD;

    /*
    pitch rate process
    */

    // pitch rate offset in deg/s
    rate_offset_pitch = _get_coordination_rate_offset_pitch(aspeed, inverted);
    rate_desired.y *= RAD_TO_DEG;
    rate_desired.y += rate_offset_pitch;
    if (gains_pitch.rmax_neg && rate_desired.y < -gains_pitch.rmax_neg) {
        rate_desired.y = -gains_pitch.rmax_neg;
    } else if (gains_pitch.rmax_pos && rate_desired.y > gains_pitch.rmax_pos) {
        rate_desired.y = gains_pitch.rmax_pos;
    }

    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = MIN(aparm.roll_limit*100 + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        rate_desired.y *= (1 - roll_prop);
    }

    rate_desired.y *= DEG_TO_RAD;

    /*
    yaw rate process
    */

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < 1.5707964f)	{
        bank_angle = constrain_float(bank_angle, -1.3962634f, 1.3962634f);
    }

    // use yaw rate offset INSTEAD of yaw control rate by NDI to ensure coordinated turn
    float rate_offset_yaw = (GRAVITY_MSS / MAX(aspeed, float(aparm.airspeed_min))) * sinf(bank_angle) * _yaw_ff;
    
    rate_desired.z = rate_offset_yaw * DEG_TO_RAD;

    // return delta_inc in degrees
    return _get_rate_out_INDI(rate_desired, aspeed, plane_shape) * RAD_TO_DEG;
}
