#include "uav_control_task.hpp"

void uavControl::uavControlTask(void) {
    if(!reset_flag) { //!reset_flag
        sim_reset_client.call(sim_reset);   
        reset_flag = sim_reset_client.call(sim_reset);
        odom_init_start_time = ros::Time::now().toSec();
    }
    else if(!odom_init_flag) {
        double now = ros::Time::now().toSec();
        double dt = now - odom_init_start_time;
        if(dt > 3) odom_init_flag=1;
    }
    else if(!takeoff_flag) {
        sim_takeoff.request.waitOnLastTask = 1;
        takeoff_flag = sim_takeoff_client.call(sim_takeoff);
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
        if(dx < 0.3 && dy < 0.3 && dz < 0.3) circle_num+=1;
        // if(circle_num==3) circle_num+=1;
        // first_target_pose.pose.position.z = 0.5;

        double drone_roll = 0, drone_pitch = 0, drone_yaw = circle_poses_true->poses.at(circle_num).yaw;
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
    
        ego_goal_point_pub.publish(first_target_pose);

        // ROS_ERROR("%f,%f,%f",posCmd.request.x,posCmd.request.y,posCmd.request.z);
        posCmd.request.x = posture_cmd.x;
        posCmd.request.y = posture_cmd.y;
        posCmd.request.z = -posture_cmd.z;
        posCmd.request.yaw = posture_cmd.yaw; 
        // ROS_ERROR("%f,%f,%f",posCmd.request.x,posCmd.request.y,posCmd.request.z);
        if (abs(posCmd.request.x) > 0.001 & abs(posCmd.request.y) > 0.001 & abs(posCmd.request.z) > 0.001) 
        {
            setGoalPosition_client.call(posCmd); //位置控制
        }
        


        // velCmd.twist = posture_cmd.uav_twist;
        // poseCmd.yaw = posture_cmd.yaw;
        // drone_vel_pub.publish(velCmd);
        // drone_pose_pub.publish(poseCmd);
    }
}  

uavControl::uavControl(ros::NodeHandle &nh) {
    posCmd.request.x = 0;
    posCmd.request.y = 0;
    posCmd.request.z = -2;
    posCmd.request.yaw = 0;
    
    circle_poses_ref_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, &uavControl::circlePosesRef_callBack,this); //仿真器输出的障碍圈参考位姿
    circle_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, &uavControl::circlePosesTrue_callBack,this); //仿真器输出的障碍圈真实位姿
    tree_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/tree_poses_gt", 10, &uavControl::treePosesTrue_callBack,this); //仿真器输出的树的位姿
    ego_pos_cmd_sub = nh.subscribe("/position_cmd", 10, &uavControl::egoPosCmd_callBack, this); //ego-planner发出的目标位姿
    ego_path_sub = nh.subscribe("/position_cmd", 10, &uavControl::egoPosCmd_callBack, this); //ego-planner发出的目标位姿
    drone_true_odom_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,&uavControl::dronePosesTrue_callBack,this);

    drone_vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);
    drone_pose_pub = nh.advertise<airsim_ros::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 10);
    setGoalPosition_client = nh.serviceClient<airsim_ros::SetLocalPosition>("/airsim_node/local_position_goal/override");
    sim_takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    sim_reset_client = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    ego_goal_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal_point", 1);
    
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
    
    // std::cout << circle_poses_ref->poses.front().position.x << std::endl;
    // std::cout << circle_poses_ref->poses.front().position.y << std::endl;
    // std::cout << circle_poses_ref->poses.front().position.z << std::endl;
} 

void uavControl::circlePosesTrue_callBack(const airsim_ros::CirclePosesConstPtr& circle) {
    circle_poses_true = circle;
    circle_msg_flag = 1;
}

void uavControl::treePosesTrue_callBack(const airsim_ros::TreePosesConstPtr& tree) {
    
}

void uavControl::egoPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds)
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
