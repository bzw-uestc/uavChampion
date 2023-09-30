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

typedef struct {
    double x,y,z,yaw;
    geometry_msgs::Twist uav_twist;
}uavPosture;

class uavControl{
private:
    //类里需要包含目标点、目标轨迹、检查是否到达目标点
    airsim_ros::SetLocalPosition posCmd;
    airsim_ros::VelCmd velCmd;
    airsim_ros::PoseCmd poseCmd;
    airsim_ros::Takeoff sim_takeoff;
    airsim_ros::Reset  sim_reset;
    ros::ServiceClient setGoalPosition_client,sim_takeoff_client,sim_reset_client;
    ros::Publisher drone_vel_pub,drone_pose_pub,drone_true_odom_pub,ego_goal_point_pub;
    ros::Subscriber drone_true_odom_sub,circle_poses_ref_sub,circle_poses_true_sub,tree_poses_true_sub; //仿真器发送的话题

    airsim_ros::CirclePosesConstPtr circle_poses_ref;   //障碍圈位姿参考值
    airsim_ros::CirclePosesConstPtr circle_poses_true;  //障碍圈位姿真值 只能debug用
    airsim_ros::TreePosesConstPtr tree_poses_true; //树的位置真值 只能debug用
    
    ros::Subscriber ego_pos_cmd_sub,ego_path_sub;
    uavPosture posture_cmd;
    int circle_num = 0;
    bool circle_msg_flag = 0;
    bool takeoff_flag = 0;
    bool reset_flag = 0;
    bool odom_init_flag = 0;
    double odom_init_start_time; //仿真器复位后的时间，里程计开始初始化的时间
    void uavSetGoalPostion(void);
    void circlePosesRef_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void circlePosesTrue_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void treePosesTrue_callBack(const airsim_ros::TreePosesConstPtr& tree);
    void egoPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds);
    // void egoPath_callBack(const visualization_msgs::Marker& pos_cmds);
    void dronePosesTrue_callBack(const geometry_msgs::PoseConstPtr& drone_poses);

public:
    uavControl(ros::NodeHandle& nh);
    ~uavControl(){}
    int drone_pose_true_flag = 0;
    geometry_msgs::PoseConstPtr drone_poses_true; //仿真器无人机真实位姿
    
    void uavControlTask(void);
};

#endif