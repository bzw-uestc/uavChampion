#include "airsim_interface.hpp"

airsimInterface::airsimInterface(ros::NodeHandle &nh) {
    circle_poses_ref_sub_ = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, &airsimInterface::circlePosesRefCallBack,this); //仿真器输出的障碍圈参考位姿
    circle_poses_true_sub_ = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, &airsimInterface::circlePosesTrueCallBack,this); //仿真器输出的障碍圈真实位姿
    drone_true_odom_sub_ = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,&airsimInterface::dronePosesTrueCallBack,this);
    sim_reset_client_ = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    set_goal_position_client_ = nh.serviceClient<airsim_ros::SetLocalPosition>("/airsim_node/local_position_goal/override");
}

bool airsimInterface::airsimReset(void) {
    sim_reset_client_.call(sim_reset_);
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
    if(circle_poses_ref_flag) {
        for(int i = 0; i < circle_poses_ref_->poses.size()-1; i++) {
            circleMsg circle_msg;
            circle_msg.pos.x = circle_poses_ref_->poses.at(i).position.x;
            circle_msg.pos.y = -circle_poses_ref_->poses.at(i).position.y;
            circle_msg.pos.z = abs(circle_poses_ref_->poses.at(i).position.z);
            circle_msg.yaw = circle_poses_ref_->poses.at(i).yaw;

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
    if(circle_poses_true_flag) {
        for(int i = 0; i < circle_poses_true_->poses.size()-1; i++) {
            circleMsg circle_msg;
            circle_msg.pos.x = circle_poses_true_->poses.at(i).position.x;
            circle_msg.pos.y = -circle_poses_true_->poses.at(i).position.y;
            circle_msg.pos.z = abs(circle_poses_true_->poses.at(i).position.z);
            circle_msg.yaw = circle_poses_true_->poses.at(i).yaw;

            circle_msg_true.push_back(circle_msg);
        }
        return circle_msg_true;
    }
    else {
        return circle_msg_true;
    }
}

void airsimInterface::dronePosesTrueCallBack(const geometry_msgs::PoseStampedConstPtr& drone_poses) {
    drone_poses_true_ = drone_poses;
    done_poses_true_flag = true;
}

void airsimInterface::circlePosesRefCallBack(const airsim_ros::CirclePosesConstPtr& circle_pose) {
    circle_poses_ref_ = circle_pose;
    circle_poses_ref_flag = true;
    // ROS_ERROR("123");
} 

void airsimInterface::circlePosesTrueCallBack(const airsim_ros::CirclePosesConstPtr& circle_true) {
    circle_poses_true_ = circle_true;
    circle_poses_true_flag = true;
}
