#ifndef _CIRCLE_TRAVEL_TASK_H
#define _CIRCLE_TRAVEL_TASK_H

#include <ros/ros.h>
#include <queue>
#include <vector>
#include <quadrotor_msgs/PositionCommand.h>
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
#include <std_msgs/Float64.h>
#include "circle_detection.hpp"
#include "../math/kalman_filter.hpp"
#include "airsim_interface.hpp"

#define ODOM_INIT_TIME 2

#define MAX_VEL_FAST_FAST 10.0
#define MAX_VEL_FAST 7.0
#define MAX_VEL_MID 5.0
#define MAX_VEL_SLOW 2.0

#define MAX_ACC_FAST_FAST 20     
#define MAX_ACC_FAST 15
#define MAX_ACC_NORMAL 10

#define CAMERA_FX 320
#define CAMERA_FY 320
#define CAMERA_CX 320
#define CAMERA_CY 240

class circleTravelTask{
private:
    airsimInterface airsim_interface_;
    ros::Publisher detect_left_pub_,ego_goal_point_pub_,drone_max_vel_pub_,drone_max_acc_pub_;
    ros::Subscriber ego_pos_cmd_sub_,visual_odom_sub_;
    std::vector<circleMsg> circle_msg_ref_; //圈的位姿参考值
    std::vector<circleMsg> circle_msg_true_; //圈的位姿真值
    std::vector<circleMsg> circle_msg_camera_; //目标检测中相机坐标系坐标
    std::vector<circleMsg> circle_msg_world_; //目标检测中世界坐标系
    std::map<int,cv::Point3f> mid_point_map_; //任务所需的中间点
    bool airsim_reset_flag_ = false;
    bool odom_init_flag_ = false;
    bool ego_init_flag_ = false;
    double odom_init_start_time_ = 0; //里程计初始化的时间
    int circle_num_ = 0; //记录当前在第几个障碍圈
    nav_msgs::Odometry drone_odom_; //无人机使用的odom
    nav_msgs::Odometry visual_odom_; //视觉里程计
    geometry_msgs::PoseStamped drone_target_pose_;
    quadrotor_msgs::PositionCommand ego_pos_cmd_;
    void droneFdbUpdate(void); //更新无人机反馈信息
    void circlePosionWorldUpdate(void); //更新障碍圈世界坐标
    void droneStateUpdate(void); //更新无人机状态 主要是最大速度、控制参数
    void droneSetGoalPosion(void); //更新无人机的目标点
    void dronePosionPDControl(void); //无人机位置PD控制
    int findClosestCircleNum(circleMsg circle_msg); //找到最靠近观测的圈的序号
    std::map<int,cv::Point3f> getMidPoint(void);
    void visualOdometryCallBack(const nav_msgs::Odometry& drone_vins_poses);
    void egoPosCmdCallBack(const quadrotor_msgs::PositionCommand& pos_cmds);
    bool droneReachedLocation(cv::Point3f circle_taget,nav_msgs::Odometry fdb,double distance_dxyz);
public:
    circleTravelTask(ros::NodeHandle& nh);
    ~circleTravelTask(){}
    circleDetection circle_detection_;
    std::queue<std::pair<std_msgs::Header,std::vector<cv::Mat>>> img_detect_buf_;
    void circleTravelMain(void);
};


#endif