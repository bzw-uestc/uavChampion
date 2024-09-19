#include "controller.hpp"
#include "../../utils/uav_utils/include/uav_utils/geometry_utils.h"
#include "../../utils/uav_utils/include/uav_utils/utils.h"

void SE3Controller::update(const Desired_State_t& des, const Odom_Data_t& odom, 
		                       Controller_Output_t& u) 
{
  Controller_Output_t reference_inputs;
  reference_inputs = computeNominalReferenceInputs(des, odom);
  // ROS_ERROR("refer: thrust:%f, yaw_rate:%f, pitch_rate:%f, roll_rate:%f",reference_inputs.normalized_thrust,reference_inputs.yaw_rate,
    // reference_inputs.pitch_rate,reference_inputs.roll_rate);
  Eigen::Vector3d e_p, e_v, F_des;
	
	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v_.setZero();
	}

  double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q); // 当前yaw航向
	Eigen::Matrix3d wRc = uav_utils::rotz(yaw_curr); // C系转世界系
	Eigen::Matrix3d cRw = wRc.transpose();
	e_p = des.p - odom.p; // 位置误差
	Eigen::Vector3d u_p = wRc * param_.Kp * cRw * e_p;  // 从右往左看， 误差转C系，乘kp，再转回世界系 
	u.des_v_real = des.v + u_p; // For estimating hover percent 用于油门估计
	e_v = des.v + u_p - odom.v;

  // 速度积分
  const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v_(k) += e_v(k) * 1.0 / 50.0;
		}
	}

  Eigen::Vector3d u_v_p = wRc * param_.Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4}; //积分限幅
	Eigen::Vector3d u_v_i = wRc * param_.Kvi * cRw * int_e_v_;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}
  Eigen::Vector3d u_v = u_v_p + u_v_i; //kp * e_p + kv * e_v + kvi * integration_v
  // 计算目标推力
  F_des = u_v * param_.mass + 
		Eigen::Vector3d(0, 0, param_.mass * param_.gra) + param_.Ka * param_.mass * des.a;
	
  Eigen::Matrix3d wRb_odom = odom.q.toRotationMatrix();
	Eigen::Vector3d z_b_curr = wRb_odom.col(2);
	double u1 = F_des.dot(z_b_curr);
	double fullparam = 1.0;
	u.thrust = u1 / param_.full_thrust;
	if(u.thrust >= fullparam)
		ROS_WARN("FULL THRUST");
	u.thrust = u.thrust >= fullparam ? fullparam:u.thrust;

  const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(F_des/param_.mass, des.yaw,odom.q);
  // Eigen::Matrix3d rotationMatrix = desired_attitude.toRotationMatrix();
  // double yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
  // double roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));  // roll: atan2(r32, r33)
  // double pitch = asin(-rotationMatrix(2, 0));
  // ROS_ERROR("desired, yaw:%f, pitch:%f, roll:%f",yaw,pitch,roll);
  
	Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude,odom.q);
  feedback_bodyrates = Eigen::Vector3d::Zero();
	u.roll_rate = reference_inputs.roll_rate + feedback_bodyrates.x();
	u.pitch_rate = reference_inputs.pitch_rate + feedback_bodyrates.y();
	u.yaw_rate = reference_inputs.yaw_rate + feedback_bodyrates.z();
	// ROS_INFO_STREAM("roll_rate: "<< u.roll_rate);
	double limit_rate = 6*3.14;
	if(u.roll_rate >= limit_rate)
		ROS_INFO("ROLL RATE LIMIT!");
	if(u.pitch_rate >= limit_rate)
		ROS_INFO("pitch_rate_limit!");

	uav_utils::limit_range(u.roll_rate,limit_rate);//3.0
	uav_utils::limit_range(u.pitch_rate,limit_rate);
	uav_utils::limit_range(u.yaw_rate,1.5);
}

void SE3Controller::update2(const Desired_State_t& des, const Odom_Data_t& odom, 
		                        Controller_Output_t& u) 
{
  std::string constraint_info("");
  Eigen::Vector3d e_p,e_v,F_des;
  double e_yaw = 0.0;

  // 积分分离
  if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v_.setZero();
	}

  double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q);
	double yaw_des = des.yaw;
	Eigen::Matrix3d wRc = uav_utils::rotz(yaw_curr);
	Eigen::Matrix3d cRw = wRc.transpose();
  e_p = des.p - odom.p;
	Eigen::Vector3d u_p = wRc * param_.Kp * cRw * e_p;
	
	e_v = des.v + u_p - odom.v;

  const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v_(k) += e_v(k) * 1.0 / 50.0;
		}
	}

  Eigen::Vector3d u_v_p = wRc * param_.Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	Eigen::Vector3d u_v_i = wRc * param_.Kvi * cRw * int_e_v_;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}

  Eigen::Vector3d u_v = u_v_p + u_v_i;

	e_yaw = yaw_des - yaw_curr;

	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);

	double u_yaw = param_.Kyaw * e_yaw;

  F_des = u_v * param_.mass + 
		Eigen::Vector3d(0, 0, param_.mass * param_.gra) + param_.Ka * param_.mass * des.a;
	
	if (F_des(2) < 0.5 * param_.mass * param_.gra)
	{
		F_des = F_des / F_des(2) * (0.5 * param_.mass * param_.gra);
	}
	else if (F_des(2) > 2 * param_.mass * param_.gra)
	{
		F_des = F_des / F_des(2) * (2 * param_.mass * param_.gra);
	}

	if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(50.0)))
	{
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(30.0));
	}

	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(50.0)))
	{
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(30.0));	
	}

  Eigen::Vector3d z_b_des = F_des / F_des.norm();
  // Z-Y-X Rotation Sequence                
	Eigen::Vector3d y_c_des = Eigen::Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
	Eigen::Vector3d x_b_des = y_c_des.cross(z_b_des) / y_c_des.cross(z_b_des).norm();
	Eigen::Vector3d y_b_des = z_b_des.cross(x_b_des);

  Eigen::Matrix3d R_des1; // it's wRb
	R_des1 << x_b_des, y_b_des, z_b_des;
	
	Eigen::Matrix3d R_des2; // it's wRb
	R_des2 << -x_b_des, -y_b_des, z_b_des;
	
	Eigen::Vector3d e1 = uav_utils::R_to_ypr(R_des1.transpose() * odom.q.toRotationMatrix());
	Eigen::Vector3d e2 = uav_utils::R_to_ypr(R_des2.transpose() * odom.q.toRotationMatrix());

	Eigen::Matrix3d R_des; // it's wRb

	if (e1.norm() < e2.norm())
	{
		R_des = R_des1;
	}
	else
	{
		R_des = R_des2;
	}

  Eigen::Vector3d F_c = wRc.transpose() * F_des;
  Eigen::Matrix3d wRb_odom = odom.q.toRotationMatrix();
  Eigen::Vector3d z_b_curr = wRb_odom.col(2);
  double u1 = F_des.dot(z_b_curr);
  double fx = F_c(0);
  double fy = F_c(1);
  double fz = F_c(2);
  u.roll  = std::atan2(-fy, fz);
  u.pitch = std::atan2( fx, fz);
  u.yaw = des.yaw;
  u.thrust = u1 / param_.full_thrust;
  
}

Controller_Output_t SE3Controller::computeNominalReferenceInputs(
  const Desired_State_t& reference_state,
  const Odom_Data_t& attitude_estimate){
  
  Controller_Output_t reference_command;
  // 根据期望yaw航向构建坐标轴C
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_state.yaw, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  // 重力前馈
  const Eigen::Vector3d des_acc = reference_state.a + Eigen::Vector3d(0,0,param_.gra);
  // 求期望姿态
  const Eigen::Quaterniond q_W_B = computeDesiredAttitude(
    des_acc, reference_state.yaw, attitude_estimate.q);

  Eigen::Matrix3d rotationMatrix = q_W_B.toRotationMatrix();
  desire_yaw_ = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)) * 57.3f;
  desire_roll_ = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2)) * 57.3f;  // roll: atan2(r32, r33)
  desire_pitch_ = asin(-rotationMatrix(2, 0)) * 57.3f;
  ROS_ERROR("des, yaw:%f, pitch:%f, roll:%f",desire_yaw_,desire_pitch_,desire_roll_);

  const Eigen::Vector3d x_B = q_W_B * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_B = q_W_B * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();

  // Reference orientation
  reference_command.orientation = q_W_B;
  // Reference thrust
  reference_command.normalized_thrust = des_acc.norm();
  // Reference body rates
  if (almostZeroThrust(reference_command.normalized_thrust)) {
    ROS_ERROR("almost_zero_thrust");
    reference_command.roll_rate = 0.0;
    reference_command.pitch_rate = 0.0;
  } 
  else {
    reference_command.roll_rate = -1.0 /
                                      reference_command.normalized_thrust *
                                      y_B.dot(reference_state.jerk);
    reference_command.pitch_rate = 1.0 /
                                      reference_command.normalized_thrust *
                                      x_B.dot(reference_state.jerk);
  }
  //   reference_command.yaw_rate = 0.0;

  if (almostZero((y_C.cross(z_B)).norm())) {
    reference_command.yaw_rate = 0.0;
  } 
  else {
    reference_command.yaw_rate =
        1.0 / (y_C.cross(z_B)).norm() *
        (reference_state.head_rate * x_C.dot(x_B) +
         reference_command.pitch_rate * y_C.dot(z_B));
  }

  return reference_command;

}

Eigen::Quaterniond SE3Controller::computeDesiredAttitude(
  const Eigen::Vector3d& desired_acceleration, const double& reference_heading,
  const Eigen::Quaterniond& attitude_estimate) const{
	// 引入当前姿态是为了防止无人机upside down 导致微分平坦歧义
	// desired_acceleration means the desired thrust and is perpendicular to the body frame.
	// 由期望heading计算出旋转四元数
	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

	// Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();  
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  Eigen::Vector3d z_B;
  if (almostZero(desired_acceleration.norm())) {
		// In case of free fall we keep the thrust direction to be the estimated one
		// This only works assuming that we are in this condition for a very short
		// time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  }	
	else {
    z_B = desired_acceleration.normalized();  // 该模型不考虑风阻，是 a + g  代表的就是z_b 具有归一化的环节
  }

	// 利用z_B 和期望航向 计算出目标姿态的x轴  引入当前姿态防止歧义
  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);  // y_C 叉积 z_B
  const Eigen::Vector3d x_B =
  computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);
	
	//目标姿态的y轴利用x轴和z轴叉积得到
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  // From the computed desired body axes we can now compose a desired attitude
	const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
	// ROS_INFO_STREAM("R_W_B: "<<R_W_B);
	const Eigen::Quaterniond desired_attitude(R_W_B);

	return desired_attitude;
}

// 该函数在x_B的模趋向0时，将y_C投影到当前姿态的的x轴上来估计x轴
Eigen::Vector3d SE3Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const {
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }

  return x_B;
}

Eigen::Vector3d SE3Controller::computeFeedBackControlBodyrates(const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate){
		  // Compute the error quaternion
  const Eigen::Quaterniond q_e = attitude_estimate.inverse() * desired_attitude;
  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;
  double krp = param_.track_Krp;
  double kyaw = param_.track_Kyaw;

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * krp * q_e.x();
    bodyrates.y() = 2.0 * krp * q_e.y();
    bodyrates.z() = 2.0 * kyaw * q_e.z();
  } else {
    bodyrates.x() = -2.0 * krp * q_e.x();
    bodyrates.y() = -2.0 * krp * q_e.y();
    bodyrates.z() = -2.0 * kyaw * q_e.z();
  }

  return bodyrates;
}

bool SE3Controller::almostZero(const double value) const{
	return fabs(value) < 0.001; 
}

bool SE3Controller::almostZeroThrust(const double thrust_value) const {
  return fabs(thrust_value) < 0.01;
}
