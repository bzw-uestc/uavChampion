#include "airsim_interface.hpp"

airsimInterface::airsimInterface(ros::NodeHandle &nh) {
    circle_poses_ref_sub_ = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, &airsimInterface::circlePosesRefCallBack,this); //仿真器输出的障碍圈参考位姿
    circle_poses_true_sub_ = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, &airsimInterface::circlePosesTrueCallBack,this); //仿真器输出的障碍圈真实位姿
    drone_true_odom_sub_ = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,&airsimInterface::dronePosesTrueCallBack,this);
    sim_reset_client_ = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    sim_takeoff_client_ = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    set_goal_position_client_ = nh.serviceClient<airsim_ros::SetLocalPosition>("/airsim_node/local_position_goal/override");
    drone_angle_rate_throttle_pub_ = nh.advertise<airsim_ros::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame",1);
}

bool airsimInterface::airsimReset(void) {
    sim_reset_client_.call(sim_reset_);
    return true;
}

bool airsimInterface::airsimTakeoff(void) {
    sim_takeoff_.request.waitOnLastTask = 1;
    sim_takeoff_client_.call(sim_takeoff_);
    return true;
}

void airsimInterface::airsimSetGoalPosition(const double x,const double y,const double z,const double yaw) {
    pos_cmd_client_.request.x = x;
    pos_cmd_client_.request.y = -y; //仿真器y和ego相反 以ego为主
    pos_cmd_client_.request.z = -z; //仿真器z和ego相反 以ego为主
    pos_cmd_client_.request.yaw = -yaw;
    set_goal_position_client_.call(pos_cmd_client_);
}

std::vector<circleMsg> airsimInterface::airsimGetCirclePosRef(void) {
    std::vector<circleMsg> circle_msg_ref;
    if(!circle_poses_ref_->poses.empty()) {
        for(int i = 0; i < circle_poses_ref_->poses.size()-1; i++) {
            circleMsg circle_msg;
            circle_msg.pos_world.x = circle_poses_ref_->poses.at(i).position.x;
            circle_msg.pos_world.y = -circle_poses_ref_->poses.at(i).position.y;
            circle_msg.pos_world.z = abs(circle_poses_ref_->poses.at(i).position.z);
            circle_msg.yaw = -circle_poses_ref_->poses.at(i).yaw / 57.3;
            circle_msg.ratio = 1.0;
            circle_msg_ref.push_back(circle_msg);
        }
        return circle_msg_ref;
    }
    else {
        return circle_msg_ref;
    }
}

std::vector<circleMsg> airsimInterface::airsimGetCirclePosTrue(void) {
    std::vector<circleMsg> circle_msg_true;
    if(!circle_poses_true_->poses.empty()) {
        for(int i = 0; i < circle_poses_true_->poses.size()-1; i++) {
            circleMsg circle_msg;
            circle_msg.pos_world.x = circle_poses_true_->poses.at(i).position.x;
            circle_msg.pos_world.y = -circle_poses_true_->poses.at(i).position.y;
            circle_msg.pos_world.z = abs(circle_poses_true_->poses.at(i).position.z);
            circle_msg.yaw = circle_poses_true_->poses.at(i).yaw;

            circle_msg_true.push_back(circle_msg);
        }
        return circle_msg_true;
    }
    else {
        return circle_msg_true;
    }
}

void airsimInterface::airsimAngleRateThrottleCtrl(const double throttle,const Eigen::Vector3d& angle_rate) {
    airsim_ros::AngleRateThrottle angleRateThrottle_cmd;
    angleRateThrottle_cmd.throttle = throttle;
    angleRateThrottle_cmd.pitchRate = angle_rate.x();
    angleRateThrottle_cmd.rollRate = angle_rate.y();
    angleRateThrottle_cmd.yawRate = angle_rate.z();
    drone_angle_rate_throttle_pub_.publish(angleRateThrottle_cmd);
}

void airsimInterface::dronePosesTrueCallBack(const geometry_msgs::PoseStampedConstPtr& drone_poses) {
    drone_poses_true_ = drone_poses;
}

void airsimInterface::circlePosesRefCallBack(const airsim_ros::CirclePosesConstPtr& circle_pose) {
    circle_poses_ref_ = circle_pose;
} 

void airsimInterface::circlePosesTrueCallBack(const airsim_ros::CirclePosesConstPtr& circle_true) {
    circle_poses_true_ = circle_true;
}
