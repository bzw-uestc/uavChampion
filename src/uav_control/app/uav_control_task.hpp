#ifndef _UAVCONTROL_H
#define _UAVCONTROL_H

#include <ros/ros.h>
#include <queue>
#include <vector>
#include <algorithm>
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
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"

// #define TF_DEBUG
#define PD_DEBUGE
// #define TRUE_POSE_DEBUGE
// #define DEBUGE1
#define ObstacleCircleRadius 0.75
#define ObstacleCircleNum    17
#define ODOM_INIT_TIME 5
#define PD_DELAY_TIME  1
#define MAX_VEL_NORMAL 3.0
#define MAX_VEL_DETECT 1.5


typedef struct {
    double x,y,z,yaw;
    geometry_msgs::Twist uav_twist;
}uavPosture;

class uavControl{
private:
    enum SelectPoint{UpperMidpoint, LowerMidpoint, LeftMidpoint, RightMidpoint};
    //类里需要包含目标点、目标轨迹、检查是否到达目标点
    airsim_ros::SetLocalPosition posCmd;
    airsim_ros::VelCmd velCmd;
    airsim_ros::PoseCmd poseCmd;
    airsim_ros::Takeoff sim_takeoff;
    airsim_ros::Reset  sim_reset;
    nav_msgs::Odometry visual_pose;
    geometry_msgs::PoseStamped ego_target_pose;
    tf::TransformListener listener;
    ros::ServiceClient setGoalPosition_client,sim_takeoff_client,sim_reset_client;
    ros::Publisher drone_vel_pub,drone_pose_pub,drone_true_odom_pub,ego_goal_point_pub,drone_max_vel_pub;
    ros::Subscriber drone_true_odom_sub,circle_poses_ref_sub,circle_poses_true_sub,tree_poses_true_sub, pd_vel_sub; //仿真器发送的话题

    airsim_ros::CirclePosesConstPtr circle_poses_ref;   //障碍圈位姿参考值
    airsim_ros::CirclePosesConstPtr circle_poses_ref_z;   //障碍圈垂直方向跳跃点
    airsim_ros::CirclePosesConstPtr circle_poses_true;  //障碍圈位姿真值 只能debug用
    airsim_ros::TreePosesConstPtr tree_poses_true; //树的位置真值 只能debug用
    
    ros::Subscriber ego_pos_cmd_sub,ego_path_sub, visual_odom_sub;
    uavPosture posture_cmd;
    int circle_num = 0;
    bool circle_msg_flag = 0;
    bool takeoff_flag = 0;
    bool reset_flag = 0;
    bool odom_init_flag = 0;
    bool visual_detect_flag = 0;
    bool pd_delay_flag = 0;
    bool aim_flag = 0;
    double pd_delay_start_time = 0;
    double odom_init_start_time = 0; //仿真器复位后的时间，里程计开始初始化的时间
    float drone_max_vel = 1.5;
    void uavSetGoalPostion(void);
    void circlePosesRef_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void circlePosesTrue_callBack(const airsim_ros::CirclePosesConstPtr& circle);
    void treePosesTrue_callBack(const airsim_ros::TreePosesConstPtr& tree);
    void egoPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds);
    void pdVelPose_callBack(const airsim_ros::VelCmdPtr& pdVel);
    // void egoPath_callBack(const visualization_msgs::Marker& pos_cmds);
    void dronePosesTrue_callBack(const geometry_msgs::PoseStampedConstPtr& drone_poses);
    void droneVisualPose_callBack(const nav_msgs::Odometry& drone_vins_poses);
    std::vector<double> detectCirclePosion(SelectPoint p, int *circleTag);
    bool uav_reached_location(geometry_msgs::PoseStamped ref,nav_msgs::Odometry fdb,double distance_dxyz);
public:
    uavControl(ros::NodeHandle& nh);
    ~uavControl(){}
    int drone_pose_true_flag = 0;
    std::vector<std::vector<float>> circle_detect_msg;  //外部vector 0存放左目结果 1存放右目结果
    geometry_msgs::PoseStampedConstPtr drone_poses_true; //仿真器无人机真实位姿
    cv::Mat image_left,image_right,image_depth,heat_map; //无人机搭载的双目图像 外部传进来
    float *pointcloud;
    void uavControlTask(void);
};

cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight);
std::vector<cv::Point2f> ORBPointsMathch(cv::Mat& image_left, cv::Mat& image_right, std::vector<float>& ROI);

#endif