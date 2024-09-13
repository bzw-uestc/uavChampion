#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

#include "CtrlParam.h"


struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d jerk;
	double yaw;
	double head_rate;
	Eigen::Quaterniond q;
};

struct Controller_Output_t
{
    static constexpr double CTRL_YAW_RATE = 1.0;
    static constexpr double CTRL_YAW = 0.0;

	double roll;
	double pitch;
	double yaw;
	double thrust;
	double roll_rate;
	double pitch_rate;
	double yaw_rate;
	double yaw_mode; // if yaw_mode > 0, CTRL_YAW;
				     // if yaw_mode < 0, CTRL_YAW_RATE
	Eigen::Quaterniond orientation;
	double normalized_thrust;

	Eigen::Vector3d des_v_real;
};

struct Odom_Data_t
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;
};

struct Ctrl_Param_t
{
    double mass;
    double gra;
    Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
    double Kyaw;
    double full_thrust;
    double track_Krp,track_Kyaw;
    double hov_percent;
};

class SE3Controller
{
private:
    Ctrl_Param_t param_;
    Eigen::Vector3d int_e_v_;
    
	double Kyaw_;
    Controller_Output_t computeNominalReferenceInputs(
        const Desired_State_t& reference_state,
        const Odom_Data_t& attitude_estimate) const;

    Eigen::Quaterniond computeDesiredAttitude(
        const Eigen::Vector3d& desired_acceleration, const double& reference_heading,
        const Eigen::Quaterniond& attitude_estimate) const;
    
    Eigen::Vector3d computeRobustBodyXAxis(
        const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
        const Eigen::Vector3d& y_C,
        const Eigen::Quaterniond& attitude_estimate) const; 

    Eigen::Vector3d computeFeedBackControlBodyrates(
        const Eigen::Quaterniond& desired_attitude,
        const Eigen::Quaterniond& attitude_estimate);

    // double get_yaw_from_quaternion(const Eigen::Quaterniond& q);
    bool almostZero(const double value) const;
    bool almostZeroThrust(const double thrust_value) const;
public:
    SE3Controller() {
        param_.mass = 0.8f;
        param_.gra = 9.8f;
        param_.Kp.setZero();
        param_.Kv.setZero();
        param_.Ka.setZero();
        param_.Kvi.setZero();
        param_.Kp(0,0) = 1.0f;
        param_.Kp(1,1) = 1.0f;
        param_.Kp(2,2) = 1.0f;
        param_.Kv(0,0) = 1.0f;
        param_.Kv(1,1) = 1.0f;
        param_.Kv(2,2) = 1.0f;
        param_.Kvi(0,0) = 0.0f;
        param_.Kvi(1,1) = 0.0f;
        param_.Kvi(2,2) = 0.0f;
        param_.Ka(0,0) = 1.0f;
        param_.Ka(1,1) = 1.0f;
        param_.Ka(2,2) = 1.0f;
        param_.Kyaw = 1.0f;
        param_.hov_percent = 0.6;
        param_.full_thrust = param_.mass * param_.gra / param_.hov_percent;
    };
    ~SE3Controller() {};
    void update(const Desired_State_t& des, const Odom_Data_t& odom, 
		        Controller_Output_t& u);
    
};

#endif