#ifndef _UAVCONTROL_H
#define _UAVCONTROL_H

#include <ros/ros.h>
#include <queue>
#include <vector>
#include <unordered_map>
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
#define ODOM_INIT_TIME 2
#define PD_DELAY_TIME  0

#define MAX_VEL_FAST 7.0
#define MAX_VEL_MID 3.0
#define MAX_VEL_SLOW 2.0
#define MAX_VEL_SLOW_SLOW 1.8
        
#define MAX_ACC_FAST 15
#define MAX_ACC_NORMAL 10

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
    ros::Publisher drone_vel_pub,drone_pose_pub,drone_true_odom_pub,ego_goal_point_pub,drone_max_vel_pub,drone_max_acc_pub;
    ros::Subscriber drone_true_odom_sub,circle_poses_ref_sub,circle_poses_true_sub,tree_poses_true_sub, pd_vel_sub; //仿真器发送的话题

    airsim_ros::CirclePosesConstPtr circle_poses_ref;   //障碍圈位姿参考值
    airsim_ros::CirclePosesConstPtr circle_poses_true;  //障碍圈位姿真值 只能debug用
    airsim_ros::TreePosesConstPtr tree_poses_true; //树的位置真值 只能debug用
    
    ros::Subscriber ego_pos_cmd_sub,ego_path_sub, visual_odom_sub;
    uavPosture posture_cmd;
    
    int circle_num = 0;
    int circle_type = 1; //圈的种类 1为红圈 0为黄圈
    
    
    bool reset_flag = false;
    bool odom_init_flag = false;
    bool ego_init_flag = false;
    bool visual_detect_flag = false;
    bool pd_delay_flag = false;
    bool aim_flag = false;
    bool circle4_flag = false;
    bool circle12_flag = false;
    bool circle13_flag = false;
    bool circle15_flag = false;
    bool circle16_obs_flag = false;  //观测确定对位点的flag
    bool circle16_arrive_flag = false;  //到达对位点的flag
    bool circle16_aim_flag = false;  //冲向动态圈的flag
    bool circle16_clash_flag = false;  //冲向动态圈的flag
    int  circle16_clash_rotation = 0;     //最高点的方向 -1为在无人机左边 1为在无人机右边
    cv::Point3f circle16_aim_point;
    cv::Point3f circle16_clash_point;  //冲向动态圈的目标点
    int circle16_rotation_flag = false;       //用参考位姿算出来圈的旋转方向 -1为向左摆  1为向右摆
    double circle16_camera_y_max = -10;
    double circle16_camera_x_max = -10;
    double circle16_camera_x_min = 10;
    double circle16_arrive_time = 0;
    double circle16_yaw = 0.0;
    double circle16_min_z = 10.0;
    double circle16_world_y_sum = 0;
    double circle16_world_y_avr = 0;
    int circle16_world_y_cnt = 0;
    cv::Point3f circle16_ref_last;
    // std::unordered_map<int,geometry_msgs::PoseStamped> mid_point_map;
    // std::unordered_map<int,int> mid_point_map;
    bool mid_point_flag = false;
    int drone_slowly_flag = 0;
    double pd_delay_start_time = 0;
    double odom_init_start_time = 0; //仿真器复位后的时间，里程计开始初始化的时间
    float drone_max_vel = MAX_VEL_FAST;
    float drone_max_acc = MAX_ACC_FAST;
    std::vector<std::vector<float>> circle_detect_msg;  //外部vector 0存放左目结果 1存放右目结果
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
    bool uav_reached_location(double target_x,double target_y,double target_z,nav_msgs::Odometry fdb,double distance_dxyz);
public:
    uavControl(ros::NodeHandle& nh);
    ~uavControl(){}
    int drone_pose_true_flag = 0;
    bool circle_msg_flag = false;
    bool takeoff_flag = false;
    std::queue<std::vector<std::vector<float>>> circle_detect_msg_queue;
    geometry_msgs::PoseStampedConstPtr drone_poses_true; //仿真器无人机真实位姿
    cv::Mat image_left,image_right,image_depth,heat_map; //无人机搭载的双目图像 外部传进来
    void uavControlTask(void);
};

cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight);
std::vector<cv::Point2f> ORBPointsMathch(cv::Mat& image_left, cv::Mat& image_right, std::vector<float>& ROI);

#endif