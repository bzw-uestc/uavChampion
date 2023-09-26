#include "uavControl.hpp"

void uavControl::uavControlTask(void) {
    if(!reset_flag) { //!reset_flag
        reset_flag = 1;
        reset_flag = sim_reset_client.call(sim_reset);
    }
    else if(!takeoff_flag) {
        takeoff.request.waitOnLastTask = 1;
        takeoff_flag = takeoff_client.call(takeoff);
    }
    else if(circle_msg_flag) { //circle_msg_flag
        geometry_msgs::PoseStamped first_target_pose;
        double dx,dy,dz;

        dx = abs(circle_poses_true->poses.at(circle_num).position.x - drone_poses_true->position.x );
        dy = abs(circle_poses_true->poses.at(circle_num).position.y - drone_poses_true->position.y );
        dz = abs(circle_poses_true->poses.at(circle_num).position.z) - abs(drone_poses_true->position.z );

        first_target_pose.pose.position.x = circle_poses_true->poses.at(circle_num).position.x ;
        first_target_pose.pose.position.y = circle_poses_true->poses.at(circle_num).position.y ;
        first_target_pose.pose.position.z = abs(circle_poses_true->poses.at(circle_num).position.z) ;
        if(dx < 0.2 && dy < 0.2 && dz < 0.2) circle_num+=1;
        // if(circle_num==3) circle_num+=1;
        // first_target_pose.pose.position.z = 0.5;

        double drone_roll = 0, drone_pitch = 0, drone_yaw = circle_poses_ref->poses.at(circle_num).yaw;
        // first_target_pose.pose.position.x = 18;
        // first_target_pose.pose.position.y = 10;
        // first_target_pose.pose.position.z = -0.5;
        // double drone_roll = 0, drone_pitch = 0, drone_yaw = 5.0;
        double c1 = cos(drone_roll / 2);
        double s1 = sin(drone_roll / 2);
        double c2 = cos(drone_pitch / 2);
        double s2 = sin(drone_pitch / 2);
        double c3 = cos(drone_yaw / 2);
        double s3 = sin(drone_yaw / 2);
        first_target_pose.pose.orientation.w = c1 * c2 * c3 + s1 * s2 * s3;
        first_target_pose.pose.orientation.x = s1 * c2 * c3 - c1 * s2 * s3;
        first_target_pose.pose.orientation.y = c1 * s2 * c3 + s1 * c2 * s3;
        first_target_pose.pose.orientation.z = c1 * c2 * s3 - s1 * s2 * c3;
    
        goal_point_pub.publish(first_target_pose);

        // ROS_ERROR("%f,%f,%f",posCmd.request.x,posCmd.request.y,posCmd.request.z);
        // posCmd.request.x = 18.0;
        // posCmd.request.y = 10.0;
        // posCmd.request.z = -0.5;
        // posCmd.request.x = 0;
        // posCmd.request.y = 0;
        // posCmd.request.z = -1.5;
        // posCmd.request.yaw = 50;
        posCmd.request.x = posture_cmd.x;
        posCmd.request.y = posture_cmd.y;
        posCmd.request.z = -posture_cmd.z;
        posCmd.request.yaw = posture_cmd.yaw;
        
        setGoalPosition_client.call(posCmd);

        // velCmd.twist = posture_cmd.uav_twist;
        // drone_vel_pub.publish(velCmd);
    }
}  

uavControl::uavControl(ros::NodeHandle &nh) {
    posCmd.request.x = 0;
    posCmd.request.y = 0;
    posCmd.request.z = -1;
    posCmd.request.yaw = 0;
    
    circle_poses_ref_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, &uavControl::circlePosesRef_callBack,this); //仿真器输出的障碍圈参考位姿
    circle_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, &uavControl::circlePosesTrue_callBack,this); //仿真器输出的障碍圈真实位姿
    tree_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/tree_poses_gt", 10, &uavControl::treePosesTrue_callBack,this); //仿真器输出的树的位姿
    ego_pos_cmd_sub = nh.subscribe("/position_cmd", 10, &uavControl::prosessPosCmd_callBack, this); //ego-planner发出的目标位姿
    drone_true_odom_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,&uavControl::dronePosesTrue_callBack,this);

    drone_vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);
    setGoalPosition_client = nh.serviceClient<airsim_ros::SetLocalPosition>("/airsim_node/local_position_goal/override");
    takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    sim_reset_client = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    drone_true_odom_pub = nh.advertise<nav_msgs::Odometry>("/goal_point", 1);
    goal_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal_point", 1);
    std::cout << "Init Finsh!!!" << std::endl;

}

void uavControl::uavSetGoalPostion(void) {

}

void uavControl::dronePosesTrue_callBack(const geometry_msgs::PoseConstPtr& drone_poses) {
    drone_pose_true_flag = 1;
    drone_poses_true = drone_poses;
}

void uavControl::circlePosesRef_callBack(const airsim_ros::CirclePosesConstPtr& circle_pose) {
    circle_poses_ref = circle_pose;
    circle_msg_flag = 1;
    // std::cout << circle_poses_ref->poses.front().position.x << std::endl;
    // std::cout << circle_poses_ref->poses.front().position.y << std::endl;
    // std::cout << circle_poses_ref->poses.front().position.z << std::endl;
} 

void uavControl::circlePosesTrue_callBack(const airsim_ros::CirclePosesConstPtr& circle) {
    circle_poses_true = circle;
}

void uavControl::treePosesTrue_callBack(const airsim_ros::TreePosesConstPtr& tree) {
    
}

void uavControl::prosessPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds)
{
    // ROS_ERROR("%f,%f,%f,%f",pos_cmds.velocity.x,pos_cmds.velocity.y,pos_cmds.velocity.z,pos_cmds.yaw_dot);
    // ROS_ERROR("%f,%f,%f,%f",pos_cmds.position.x,pos_cmds.position.y,pos_cmds.position.z,pos_cmds.yaw);
    posture_cmd.x = pos_cmds.position.x;
    posture_cmd.y = pos_cmds.position.y;
    posture_cmd.z = pos_cmds.position.z;
    posture_cmd.yaw = pos_cmds.yaw;
    
    posture_cmd.uav_twist.linear.x = pos_cmds.velocity.x;
    posture_cmd.uav_twist.linear.y = pos_cmds.velocity.y;
    posture_cmd.uav_twist.linear.z = -pos_cmds.velocity.z;
    posture_cmd.uav_twist.angular.x = pos_cmds.yaw_dot;
}
