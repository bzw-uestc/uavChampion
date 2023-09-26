#ifndef _UAVCONTROL_H
#define _UAVCONTROL_H

#include <ros/ros.h>
#include <queue>
#include <vector>
#include <airsim_ros/VelCmd.h>
#include <airsim_ros/SetLocalPosition.h>
#include <airsim_ros/Takeoff.h>
#include <airsim_ros/PoseCmd.h>
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "airsim_ros/CirclePoses.h"
#include "airsim_ros/TreePoses.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// template<typename T>
// class MaxSizeQueue {
// private:
//     std::queue<T> queue_;
//     size_t max_size_;

// public:
//     MaxSizeQueue(size_t max_size) : max_size_(max_size) {}

//     void push(const T& value) {
//         if (queue_.size() >= max_size_) {
//             queue_.pop();
//         }
//         queue_.push(value);
//     }

//     T front() const {
//         return queue_.front();
//     }

//     size_t size() const {
//         return queue_.size();
//     }

//     bool empty() const {
//         return queue_.empty();
//     }
//     T pop() const {
//         return queue_.pop();
//     }
//     T pop() const {
//         return queue_.pop();
//     }
//     void emplace(&color_msg_p) {
//         queue_.emplace(color_msg_p);
//     }

// };

typedef struct {
    double x,y,z,yaw;
    geometry_msgs::Twist uav_twist;
}uavPosture;

class uavControl{
private:
    //类里需要包含目标点、目标轨迹、检查是否到达目标点
    airsim_ros::SetLocalPosition posCmd;
    airsim_ros::VelCmd velCmd;
    ros::ServiceClient setGoalPosition_client;
    airsim_ros::Takeoff takeoff;
    ros::ServiceClient takeoff_client;
    airsim_ros::Reset  sim_reset;
    ros::ServiceClient sim_reset_client;
    ros::Publisher drone_vel_pub,drone_true_odom_pub,goal_point_pub;
    ros::Subscriber drone_true_odom_sub;
    ros::Subscriber circle_poses_ref_sub,circle_poses_true_sub,tree_poses_true_sub;
    airsim_ros::CirclePosesConstPtr circle_poses_ref;   //障碍圈位姿参考值
    airsim_ros::CirclePosesConstPtr circle_poses_true;  //障碍圈位姿真值 只能debug用
    airsim_ros::TreePosesConstPtr tree_poses_true; //树的位置真值 只能debug用
    

    ros::Subscriber ego_pos_cmd_sub;
    uavPosture posture_cmd;
    int circle_num = 0;
    bool circle_msg_flag = 0;
    bool takeoff_flag = 0;
    bool reset_flag = 0;
    void uavSetGoalPostion(void);
    void circlePosesRef_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void circlePosesTrue_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void treePosesTrue_callBack(const airsim_ros::TreePosesConstPtr& tree);
    void prosessPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds);
    void dronePosesTrue_callBack(const geometry_msgs::PoseConstPtr& drone_poses);
    
public:
    uavControl(ros::NodeHandle& nh);
    ~uavControl(){}
    int drone_pose_true_flag = 0;
    geometry_msgs::PoseConstPtr drone_poses_true; //仿真器无人机真实位姿
    
    void uavControlTask(void);
};

#endif