#ifndef _AIRSIM_INTERFACE_H
#define _AIRSIM_INTERFACE_H

#include <ros/ros.h>
#include <airsim_ros/VelCmd.h>
#include <airsim_ros/SetLocalPosition.h>
#include <airsim_ros/Takeoff.h>
#include <airsim_ros/PoseCmd.h>
#include <airsim_ros/Reset.h>
#include <airsim_ros/Land.h>
#include <airsim_ros/GPSYaw.h>
#include <airsim_ros/CirclePoses.h>
#include <airsim_ros/TreePoses.h>
#include <airsim_ros/AngleRateThrottle.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include "circle_detection.hpp"

class airsimInterface{
private:
    airsim_ros::SetLocalPosition posCmd_;
    airsim_ros::Reset  sim_reset_;
    airsim_ros::Takeoff sim_takeoff_;
    airsim_ros::SetLocalPosition pos_cmd_client_;
    ros::ServiceClient set_goal_position_client_,sim_reset_client_,sim_takeoff_client_;
    airsim_ros::TreePosesConstPtr tree_poses_true_; //树的位置真值 只能debug用
    
    airsim_ros::CirclePosesConstPtr circle_poses_ref_;   //障碍圈位姿参考值
    airsim_ros::CirclePosesConstPtr circle_poses_true_;  //障碍圈位姿真值 只能debug用
    ros::Subscriber drone_true_odom_sub_,circle_poses_ref_sub_,circle_poses_true_sub_; //仿真器发送的话题
    ros::Publisher drone_angle_rate_throttle_pub_; 
    void dronePosesTrueCallBack(const geometry_msgs::PoseStampedConstPtr& drone_poses);
    void circlePosesRefCallBack(const airsim_ros::CirclePosesConstPtr& circle_pose);
    void circlePosesTrueCallBack(const airsim_ros::CirclePosesConstPtr& circle);
public:
    airsimInterface(ros::NodeHandle &nh);
    ~airsimInterface(){}
    bool airsimReset(void);
    bool airsimTakeoff(void);
    void airsimSetGoalPosition(const double x,const double y,const double z,const double yaw);
    void airsimAngleRateThrottleCtrl(const double throttle,const Eigen::Vector3d& angle_rate);
    std::vector<circleMsg> airsimGetCirclePosRef(void);
    std::vector<circleMsg> airsimGetCirclePosTrue(void);
    geometry_msgs::PoseStampedConstPtr drone_poses_true_; //仿真器无人机真实位姿
    // bool airsimGetDronePosTrue();
};
#endif