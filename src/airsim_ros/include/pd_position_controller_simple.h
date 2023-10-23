#ifndef _PID_POSITION_CONTROLLER_SIMPLE_H_
#define _PID_POSITION_CONTROLLER_SIMPLE_H_


#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <airsim_ros/VelCmd.h>
#include <airsim_ros/SetLocalPosition.h>
#include <airsim_ros/SetGPSPosition.h>
#include <airsim_ros/GPSYaw.h>
#include <math_common.h>
#include <utils.h>
#include "std_msgs/Float64.h"

// #define TRUE_POSE_DEBUGE
    // todo nicer api
class PIDParams
{
public:
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;
    double kd_x;
    double kd_y;
    double kd_z;
    double kd_yaw;

    double reached_thresh_xyz;
    double reached_yaw_degrees;

    PIDParams()
        : kp_x(0.8), kp_y(0.8), kp_z(0.8), kp_yaw(0.5), kd_x(0.5), kd_y(0.5), kd_z(0.5), kd_yaw(0.2), reached_thresh_xyz(0.5), reached_yaw_degrees(0.5)
    {
    }

};

// todo should be a common representation
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

// todo should be a common representation
class DynamicConstraints
{
public:
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;

    DynamicConstraints()
        : max_vel_horz_abs(2.0), max_vel_vert_abs(2.0), max_yaw_rate_degree(1)
    {
    }

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

class PIDPositionController
{
public:
    PIDPositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    // ROS service callbacks
    bool local_position_goal_srv_cb(airsim_ros::SetLocalPosition::Request& request, airsim_ros::SetLocalPosition::Response& response);
    bool local_position_goal_srv_override_cb(airsim_ros::SetLocalPosition::Request& request, airsim_ros::SetLocalPosition::Response& response);
    bool gps_goal_srv_cb(airsim_ros::SetGPSPosition::Request& request, airsim_ros::SetGPSPosition::Response& response);
    bool gps_goal_srv_override_cb(airsim_ros::SetGPSPosition::Request& request, airsim_ros::SetGPSPosition::Response& response);

    // ROS subscriber callbacks
    void airsim_odom_cb(const geometry_msgs::PoseStamped& odom_msg);
    void visual_odom_cb(const nav_msgs::Odometry& odom_msg);
    void max_vel_cb(const std_msgs::Float64 max_vel);
    //void home_geopoint_cb(const airsim_ros::GPSYaw& gps_msg);

    void update_control_cmd_timer_cb(const ros::TimerEvent& event);

    void reset_errors();

    void initialize_ros();
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void publish_control_cmd();
    void check_reached_goal();
    
private:
    // msr::airlib::GeodeticConverter geodetic_converter_;
    static constexpr bool use_eth_lib_for_geodetic_conv_ = true;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    DynamicConstraints constraints_;
    PIDParams params_;
    XYZYaw target_position_;
    XYZYaw curr_position_;
    XYZYaw prev_error_;
    XYZYaw curr_error_;

    bool has_home_geo_;
    airsim_ros::GPSYaw gps_home_msg_;

    double update_control_every_n_sec;

    geometry_msgs::PoseStamped curr_odom_;
    airsim_ros::VelCmd vel_cmd_;
    bool reached_goal_;
    bool has_goal_;
    bool has_odom_;
    bool got_goal_once_;
    // todo check for odom msg being older than n sec

    // ros::Publisher airsim_vel_cmd_world_frame_pub_;
    ros::Publisher airsim_vel_cmd_body_frame_pub_;
    ros::Subscriber airsim_odom_sub_;
    ros::Subscriber visual_odom_sub_;
    ros::Subscriber drone_max_vel_sub_;
    ros::Subscriber home_geopoint_sub_;
    ros::ServiceServer local_position_goal_srvr_;
    ros::ServiceServer local_position_goal_override_srvr_;
    ros::ServiceServer gps_goal_srvr_;
    ros::ServiceServer gps_goal_override_srvr_;

    ros::Timer update_control_cmd_timer_;
};

#endif /* _PID_POSITION_CONTROLLER_SIMPLE_ */
