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
        mid_point_flag = false;
        if(dt > 3) odom_init_flag=1;
    }
    else if(!takeoff_flag) {
        sim_takeoff.request.waitOnLastTask = 1;
        takeoff_flag = sim_takeoff_client.call(sim_takeoff);
    }
    else if(circle_msg_flag) { //circle_msg_flag
        double dx,dy,dz;
        #ifdef DEBUGE1
        dx = abs(circle_poses_true->poses.at(circle_num).position.x - drone_poses_true->position.x );
        dy = abs(circle_poses_true->poses.at(circle_num).position.y - drone_poses_true->position.y );
        dz = abs(circle_poses_true->poses.at(circle_num).position.z) - abs(drone_poses_true->position.z );
        #else
        // ROS_ERROR("ego_target_pose:%f,%f,%f",ego_target_pose.pose.position.x,ego_target_pose.pose.position.y,ego_target_pose.pose.position.z);
        // ROS_ERROR("visual_pose:%f,%f,%f",visual_pose.pose.pose.position.x,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
        dx = abs(ego_target_pose.pose.position.x - visual_pose.pose.pose.position.x );
        dy = abs(-ego_target_pose.pose.position.y - visual_pose.pose.pose.position.y );
        dz = abs(-ego_target_pose.pose.position.z - visual_pose.pose.pose.position.z );
        #endif
        if(dx < 0.3 && dy < 0.3 && dz < 0.3)
        {
            if (mid_point_flag == false)
            {
                circle_num+=1;
                visual_detect_flag = 0;
                // mid_point_flag = true;
            }
            // else{
            //     mid_point_flag = false;
            // }
            
        }

        cv::Point3f circlePosition;
        if(!circle_detect_msg.empty()) { //当检测信息不为空时尝试恢复检测目标的3D坐标
            cv::Point3f upm = detectCirclePosion(UpperMidpoint);
            cv::Point3f lom = detectCirclePosion(LowerMidpoint);
            cv::Point3f lem = detectCirclePosion(LeftMidpoint);
            cv::Point3f rim = detectCirclePosion(RightMidpoint);
            // circlePosition = upm;
            if ((upm.x != 0) && (lom.x != 0) && (lem.x != 0) && (rim.x != 0))
            {
                circlePosition.x = (upm.x + lom.x + lem.x + rim.x) / 4;
                circlePosition.y = (upm.y + lom.y + lem.y + rim.y) / 4;
                circlePosition.z = (upm.z + lom.z + lem.z + rim.z) / 4;
                ROS_ERROR("upm:%f,%f,%f",upm.x ,upm.y,upm.z);
                ROS_ERROR("lom:%f,%f,%f",lom.x ,lom.y,lom.z);
                ROS_ERROR("upm:%f,%f,%f",lem.x ,lem.y,lem.z);
                ROS_ERROR("lom:%f,%f,%f",rim.x ,rim.y,rim.z);
            }
            
            // circlePosition.x = (lem.x + rim.x) / 2;
            // circlePosition.y = (lem.y + rim.y) / 2;
            // circlePosition.z = (lem.z + rim.z) / 2;
        }
        if(visual_detect_flag && (mid_point_flag == false)) { //进入视觉模式
            if(circlePosition.x!=0 && circlePosition.y!=0 && circlePosition.z!=0) {
                ego_target_pose.pose.position.x = circlePosition.x;
                ego_target_pose.pose.position.y = -circlePosition.y;
                ego_target_pose.pose.position.z = circlePosition.z;
            }
        }
        else if ((visual_detect_flag == false) && (mid_point_flag == false)){
            ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
            ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
            ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            
            double drone_roll = 0, drone_pitch = 0, drone_yaw = circle_poses_ref->poses.at(circle_num).yaw;
            double c1 = cos(drone_roll / 2);
            double s1 = sin(drone_roll / 2);
            double c2 = cos(drone_pitch / 2);
            double s2 = sin(drone_pitch / 2);
            double c3 = cos(drone_yaw / 2);
            double s3 = sin(drone_yaw / 2);
            ego_target_pose.pose.orientation.w = c1 * c2 * c3 + s1 * s2 * s3;
            ego_target_pose.pose.orientation.x = s1 * c2 * c3 - c1 * s2 * s3;
            ego_target_pose.pose.orientation.y = c1 * s2 * c3 + s1 * c2 * s3;
            ego_target_pose.pose.orientation.z = c1 * c2 * s3 - s1 * s2 * c3;
        }
        else{
            // mid_point_flag = true;
            if (circle_num == 0)
            {
                ego_target_pose.pose.position.x = 0;
                ego_target_pose.pose.position.y = 0;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else{
                ego_target_pose.pose.position.x = (circle_poses_ref->poses.at(circle_num - 1).position.x + circle_poses_ref->poses.at(circle_num).position.x) / 2;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num - 1).position.y + circle_poses_ref->poses.at(circle_num).position.y ) / 2;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
                double drone_roll = 0, drone_pitch = 0, drone_yaw = circle_poses_ref->poses.at(circle_num).yaw;
                double c1 = cos(drone_roll / 2);
                double s1 = sin(drone_roll / 2);
                double c2 = cos(drone_pitch / 2);
                double s2 = sin(drone_pitch / 2);
                double c3 = cos(drone_yaw / 2);
                double s3 = sin(drone_yaw / 2);
                ego_target_pose.pose.orientation.w = c1 * c2 * c3 + s1 * s2 * s3;
                ego_target_pose.pose.orientation.x = s1 * c2 * c3 - c1 * s2 * s3;
                ego_target_pose.pose.orientation.y = c1 * s2 * c3 + s1 * c2 * s3;
                ego_target_pose.pose.orientation.z = c1 * c2 * s3 - s1 * s2 * c3;
            }
            
        }
        ego_target_pose.header.frame_id = "world";
        ego_target_pose.pose.orientation.w = 1;
        ego_target_pose.pose.orientation.x = 0;
        ego_target_pose.pose.orientation.y = 0;
        ego_target_pose.pose.orientation.z = 0;
        ego_goal_point_pub.publish(ego_target_pose);
        // ROS_ERROR("circle:%f,%f,%f",visual_pose.pose.pose.position.x ,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
        
        // posCmd.request.x = posture_cmd.x;
        // posCmd.request.y = -posture_cmd.y;
        // posCmd.request.z = -posture_cmd.z;
        // posCmd.request.x = 2;
        // posCmd.request.y = 2;
        // posCmd.request.z = -2;
        // posCmd.request.yaw = posture_cmd.yaw;
        // if (abs(posCmd.request.x) > 0.001 & abs(posCmd.request.y) > 0.001 & abs(posCmd.request.z) > 0.001)
        // {
        //     setGoalPosition_client.call(posCmd); //位置控制
        // }
        
        // if(ego_receive_flag) {
            
            
            #ifdef PD_DEBUGE
                posCmd.request.x = posture_cmd.x;
                posCmd.request.y = -posture_cmd.y;
                posCmd.request.z = -posture_cmd.z;
                posCmd.request.yaw = -posture_cmd.yaw;
                // posCmd.request.yaw = 0;
                setGoalPosition_client.call(posCmd);
            #else

                velCmd.twist = posture_cmd.uav_twist;
                poseCmd.yaw = posture_cmd.yaw;
                poseCmd.roll = posture_cmd.yaw;
                poseCmd.pitch = posture_cmd.yaw;
                
                drone_vel_pub.publish(velCmd);
                drone_pose_pub.publish(poseCmd);
            #endif
            ego_receive_flag = 0;

            tf_test_pose.header.frame_id = "world";
            // double test_yaw = 30;
            // double test_pitch = 0;
            // double test_roll = 0;
            // tf::Quaternion quat;
            // tf::Matrix3x3(quat).getRPY(test_yaw, test_pitch, test_roll);//进行转换
            // tf_test_pose.pose.orientation.w = quat.getW();
            // tf_test_pose.pose.orientation.x = quat.getX();
            // tf_test_pose.pose.orientation.y = quat.getY();
            // tf_test_pose.pose.orientation.z = quat.getZ();
            tf_modey_pub.publish(tf_test_pose);

        // }
        // else {
        //     airsim_ros::VelCmd velCmdZero;
        //     velCmd = velCmdZero;
        //     drone_vel_pub.publish(velCmd);
        //     drone_pose_pub.publish(poseCmd);
        // }
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
    // visual_odom_sub = nh.subscribe("/vins_fusion/imu_propagate",1,&uavControl::droneVisualPose_callBack,this);
    visual_odom_sub = nh.subscribe("/vins_fusion/imu_propagate_for_pd",1,&uavControl::droneVisualPose_callBack,this);
    pd_vel_sub = nh.subscribe("/airsim_node/drone_1/vel_cmd_body_frame_yaw",1,&uavControl::pdVelPose_callBack,this);

    drone_vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);
    drone_pose_pub = nh.advertise<airsim_ros::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 10);
    tf_modey_pub = nh.advertise<geometry_msgs::PoseStamped>("body_tf_test", 10);
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
    
    // ROS_ERROR("TRUE:%f,%f,%f, %f", drone_poses_true->orientation.w,drone_poses_true->orientation.x,drone_poses_true->orientation.y,drone_poses_true->orientation.z);
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

void uavControl::egoPosCmd_callBack(const quadrotor_msgs::PositionCommand& pos_cmds)
{
    // ROS_ERROR("%f,%f,%f,%f",pos_cmds.velocity.x,pos_cmds.velocity.y,pos_cmds.velocity.z,pos_cmds.yaw_dot);
    // ROS_ERROR("%f,%f,%f,%f",pos_cmds.position.x,pos_cmds.position.y,pos_cmds.position.z,pos_cmds.yaw);
    ego_receive_flag = true;
    posture_cmd.x = pos_cmds.position.x;
    posture_cmd.y = pos_cmds.position.y;
    posture_cmd.z = pos_cmds.position.z;
    posture_cmd.yaw = pos_cmds.yaw;
    // posture_cmd.yaw = 0;
    posture_cmd.uav_twist.linear.x = pos_cmds.velocity.x;
    posture_cmd.uav_twist.linear.y = -pos_cmds.velocity.y;
    posture_cmd.uav_twist.linear.z = -pos_cmds.velocity.z;
    // posture_cmd.uav_twist.angular.z = -pos_cmds.yaw_dot;
    // posture_cmd.uav_twist.linear.x = 0;
    // posture_cmd.uav_twist.linear.y = 0;
    // posture_cmd.uav_twist.linear.z = 0;
    // if (abs(pos_cmds.yaw_dot) > 0.1)
    // {
        // posture_cmd.uav_twist.angular.z = pos_cmds.yaw_dot;
    // }
    
//     ROS_ERROR("ego-planner-target-yaw:%f",posture_cmd.yaw);
//     ROS_ERROR("ego-planner-target-dot:%f",posture_cmd.uav_twist.angular.z);
}

void uavControl::pdVelPose_callBack(const airsim_ros::VelCmdPtr& pdVel)
{
    // posture_cmd.uav_twist.angular.z = pdVel->twist.angular.z;
    // ROS_ERROR("YAW_DOT:%f",posture_cmd.uav_twist.angular.z);
}

void uavControl::droneVisualPose_callBack(const nav_msgs::Odometry& drone_vins_poses){
    visual_pose = drone_vins_poses;
}

cv::Point3f uavControl::detectCirclePosion(SelectPoint p)
{
    #ifdef TF_DEBUG
        cv::Point3f body_frame_test;
        body_frame_test.x = 2;
        body_frame_test.y = 4;
        body_frame_test.z = 1;

        Eigen::Quaterniond quaternion(visual_pose.pose.pose.orientation.w, visual_pose.pose.pose.orientation.x, visual_pose.pose.pose.orientation.y, visual_pose.pose.pose.orientation.z);
        Eigen::Vector3d translation(visual_pose.pose.pose.position.x, visual_pose.pose.pose.position.y, visual_pose.pose.pose.position.z);
        Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
        Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        Twb.block<3, 1>(0, 3) = translation;
        // 创建一个包含原始点坐标的齐次坐标向量
        Eigen::Vector4d Pc;
        // Pc << body_frame_test.z, body_frame_test.y, -body_frame_test.x, 1.0;
        Pc << body_frame_test.x, body_frame_test.y, body_frame_test.z, 1.0;
        Eigen::Matrix4d Tbc;
        Tbc << 0.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        // Tbc <<
        // -1.2011755535951574e-02, -9.9989771278249839e-01,-7.7641291381007780e-03, -1.7537934807651159e-01,
        //  9.9991479583090936e-01, -1.2050922069499759e-02, 5.0176045784532473e-03, -7.4810113937173170e-02,
        // -5.1106562568231781e-03, -7.7031973623564020e-03, 9.9995727005858659e-01,  5.5546220644694155e-01,
        // 0.0, 0.0, 0.0, 1,0;
        // 使用变换矩阵 T 进行坐标变换
        Eigen::Vector4d Pw = Twb * Pc;
        // 提取变换后的世界坐标系下的坐标
        tf_test_pose.pose.position.x = Pw(0);
        tf_test_pose.pose.position.y = Pw(1);
        tf_test_pose.pose.position.z = Pw(2);
        // circlePosition.x = point_world.point.x;
        // circlePosition.y = -point_world.point.y;
        // circlePosition.z = point_world.point.z;

        // ROS_ERROR("vins:%f,%f,%f",visual_pose.pose.pose.position.x ,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
        // ROS_ERROR("CircleTruePose:%f,%f,%f",circle_poses_true->poses.front().position.x ,circle_poses_true->poses.front().position.y,circle_poses_true->poses.front().position.z);
        // ROS_ERROR("point3DWorld:%f,%f,%f",circlePosition.x ,circlePosition.y,circlePosition.z);
        return body_frame_test;
    #else
        cv::Point3f circlePosition;
        // ROS_ERROR("detect0:%f,%f,%f,%f,%f",circle_detect_msg[0][0],circle_detect_msg[0][1],circle_detect_msg[0][2],circle_detect_msg[0][3],circle_detect_msg[0][4]);
        // ROS_ERROR("detect1:%f,%f,%f,%f,%f",circle_detect_msg[1][0],circle_detect_msg[1][1],circle_detect_msg[1][2],circle_detect_msg[1][3],circle_detect_msg[1][4]);
        int circleNum[2],circleTag[2]; //检测圈的个数、最大面积对应的是第几个障碍圈
        double circleSquareMax[2]; //检测圈的最大面积
        for(int i=0; i < circle_detect_msg.size(); i++) { //遍历双目的检测信息 计算面积
            circleNum[i] = circle_detect_msg[i].size() / 5; //检测结果存放的信息为：左上角x坐标、左上角y坐标、宽度、高度、类别名
            for(int j=0; j<circleNum[i]; j++) {
                double squareTemp = circle_detect_msg[i][j*5+2] * circle_detect_msg[i][j*5+3];
                if(squareTemp > circleSquareMax[i]) {
                    circleSquareMax[i] = squareTemp;
                    circleTag[i] = j;  //标记第几个圈是最大面积圈
                }
            }
            // ROS_ERROR("detect%d:%f,%f,%f,%f,%f",i,circle_detect_msg[i][0],circle_detect_msg[i][1],circle_detect_msg[i][2],circle_detect_msg[i][3],circle_detect_msg[i][4]);
        }

            

        // visual_detect_flag = 0;
        // ROS_ERROR("squaremax:%f,%f",circleSquareMax[0],circleSquareMax[1]);
        // ROS_ERROR("square:%f,%f,%f,%f,%f",circle_detect_msg[0],circle_detect_msg[1],circle_detect_msg[2],circle_detect_msg[3],circle_detect_msg[4]);
        if (circleSquareMax[0] > 3500.0 && circleSquareMax[1] > 3500.0 && circleSquareMax[0] < 12000.0 && circleSquareMax[1] < 12000.0){  //矩形面积大于4000时计算矩形框内像素点的3D空间坐标位置
            // ORBPointsMathch(image_left,image_right,circle_detect_msg[0]);  //该方法先不用 等我回来调试
            visual_detect_flag = 1;
            int x1_left, y1_left, x1_right, y1_right;
            cv::Point3f worldPoint_upm,worldPoint_lom,worldPoint_lem,worldPoint_rim;
            cv::Point3f worldPoint;
            if ( p == UpperMidpoint)
            {
                x1_left = circle_detect_msg[0][circleTag[0]*5] + 0.5 *circle_detect_msg[0][circleTag[0]*5+2]; //左上角x坐标+宽度的一半
                y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 5; //左上角y坐标
                x1_right = circle_detect_msg[1][circleTag[1]*5] + 0.5 *circle_detect_msg[1][circleTag[1]*5+2];
                y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 5;
                
                // ROS_ERROR("left:%f,right:%f",circleSquareMax[0],circleSquareMax[1]);
                // ROS_ERROR("left:%d,%d,right:%d,%d",x1_left,y1_left,x1_right,y1_right);
                //以矩形框的中间点计算3D空间坐标
                cv::Point2f l(x1_left , y1_left);
                cv::Point2f r(x1_right, y1_right);
                worldPoint_upm = uv2xyz(l, r);  //由左右目像素坐标恢复空间3D坐标
                worldPoint = worldPoint_upm;
                // ROS_ERROR("point3D:%f,%f,%f",worldPoint_upm.x,worldPoint_upm.y,worldPoint_upm.z);

                cv::Point point1_left(x1_left, y1_left);
                cv::circle(image_left, point1_left, 3, cv::Scalar(0, 255, 120), -1);
                cv::Point point1_right(x1_right, y1_right);
                cv::circle(image_right, point1_right, 3, cv::Scalar(0, 255, 120), -1);

            }
            else if (p == LowerMidpoint)
            {
                x1_left = circle_detect_msg[0][circleTag[0]*5] + 0.5 *circle_detect_msg[0][circleTag[0]*5+2]; //左上角x坐标+宽度的一半
                y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + circle_detect_msg[0][circleTag[0]*5+3] - 5; //左下角y坐标
                x1_right = circle_detect_msg[1][circleTag[1]*5] + 0.5 *circle_detect_msg[1][circleTag[1]*5+2];
                y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + circle_detect_msg[1][circleTag[1]*5+3] - 5;

                cv::Point2f l(x1_left , y1_left);
                cv::Point2f r(x1_right, y1_right);
                worldPoint_lom = uv2xyz(l, r);  //由左右目像素坐标恢复空间3D坐标
                worldPoint = worldPoint_lom;


                cv::Point point1_left(x1_left, y1_left);
                cv::circle(image_left, point1_left, 3, cv::Scalar(0, 255, 120), -1);
                cv::Point point1_right(x1_right, y1_right);
                cv::circle(image_right, point1_right, 3, cv::Scalar(0, 255, 120), -1);
                
            }
            else if (p == LeftMidpoint)
            {
                x1_left = circle_detect_msg[0][circleTag[0]*5] + 5; //左上角x坐标
                y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 0.5 * circle_detect_msg[0][circleTag[0]*5+3]; //左上角y坐标 - 高度一半
                x1_right = circle_detect_msg[1][circleTag[1]*5] + 5;
                y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 0.5 * circle_detect_msg[1][circleTag[1]*5+3] ;
                // ROS_ERROR("left:%f,right:%f",circleSquareMax[0],circleSquareMax[1]);
                // ROS_ERROR("left:%d,%d,right:%d,%d",x1_left,y1_left,x1_right,y1_right);
                //以矩形框的中间点计算3D空间坐标
                cv::Point2f l(x1_left , y1_left);
                cv::Point2f r(x1_right, y1_right);
                worldPoint_lem = uv2xyz(l, r);  //由左右目像素坐标恢复空间3D坐标
                worldPoint = worldPoint_lem;
                // ROS_ERROR("point3D:%f,%f,%f",worldPoint2.x,worldPoint2.y,worldPoint2.z);

                cv::Point point1_left(x1_left, y1_left);
                cv::circle(image_left, point1_left, 3, cv::Scalar(0, 255, 120), -1);
                cv::Point point1_right(x1_right, y1_right);
                cv::circle(image_right, point1_right, 3, cv::Scalar(0, 255, 120), -1);
            }
            else if (p == RightMidpoint)
            {
                x1_left = circle_detect_msg[0][circleTag[0]*5] + circle_detect_msg[0][circleTag[0]*5+2] - 5; //左上角x坐标+宽度
                y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 0.5 * circle_detect_msg[0][circleTag[0]*5+3] ; 
                x1_right = circle_detect_msg[1][circleTag[1]*5] + circle_detect_msg[1][circleTag[1]*5+2] - 5;
                y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 0.5 * circle_detect_msg[1][circleTag[1]*5+3];


                cv::Point2f l(x1_left , y1_left);
                cv::Point2f r(x1_right, y1_right);
                worldPoint_rim = uv2xyz(l, r);  //由左右目像素坐标恢复空间3D坐标
                worldPoint = worldPoint_rim;

                cv::Point point1_left(x1_left, y1_left);
                cv::circle(image_left, point1_left, 3, cv::Scalar(0, 255, 120), -1);
                cv::Point point1_right(x1_right, y1_right);
                cv::circle(image_right, point1_right, 3, cv::Scalar(0, 255, 120), -1);
            }
            // ROS_ERROR("x1_left:%d,y1_left:%d,x1_right:%d,y1_right:%d",x1_left,y1_left,x1_right,y1_right);
            // cv::imshow("result_left", image_left);
            // cv::imshow("result_right", image_right);
            // cv::waitKey(1);

            // cv::imwrite("/home/uestc/bzw_ws/1.jpg", image_left);
            
            
            // geometry_msgs::PointStamped point_camera;
            // point_camera.header.frame_id = "camera";
            // point_camera.point.x = worldPoint.x;
            // point_camera.point.y = worldPoint.y;
            // point_camera.point.z = worldPoint.z;
            // geometry_msgs::PointStamped point_world;
            // listener.transformPoint("world", point_camera, point_world);
            
            if(worldPoint.x <10.0 && worldPoint.y <10.0 && worldPoint.z <10.0) {
            
                // ROS_INFO("Point in camera frame: (%.2f, %.2f, %.2f)", point_camera.point.x, point_camera.point.y, point_camera.point.z);
                // ROS_ERROR("Point in world frame: (%.2f, %.2f, %.2f)", point_world.point.x, point_world.point.y, point_world.point.z);
                /*获取视觉里程计到世界坐标系的变换矩阵*/
                Eigen::Quaterniond quaternion(visual_pose.pose.pose.orientation.w, visual_pose.pose.pose.orientation.x, visual_pose.pose.pose.orientation.y, visual_pose.pose.pose.orientation.z);
                Eigen::Vector3d translation(visual_pose.pose.pose.position.x, visual_pose.pose.pose.position.y, -visual_pose.pose.pose.position.z);
                // Eigen::Quaterniond quaternion(drone_poses_true->orientation.w, drone_poses_true->orientation.x, drone_poses_true->orientation.y, drone_poses_true->orientation.z);
                // Eigen::Vector3d translation(drone_poses_true->position.x, drone_poses_true->position.y, -drone_poses_true->position.z);
                Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                Twb.block<3, 1>(0, 3) = translation;
                // 创建一个包含原始点坐标的齐次坐标向量
                Eigen::Vector4d Pc;
                // if ( p == UpperMidpoint)
                // {
                    
                //     if (circle_num == 1)
                //     {
                //         #ifdef PD_DEBUGE
                //             Pc << worldPoint1.z, worldPoint1.y + 0.7, -worldPoint1.x , 1.0;
                //         #else
                //             Pc << worldPoint1.z, worldPoint1.y, (worldPoint1.x - ObstacleCircleRadius), 1.0;
                //         #endif
                        
                //         ROS_ERROR("DEBUG");
                //     }
                //     else{
                //          #ifdef PD_DEBUGE
                //             Pc << worldPoint1.z, worldPoint1.y +0.7, -(worldPoint1.x + 0.85), 1.0;
                //          #else
                //             Pc << worldPoint1.z, worldPoint1.y, worldPoint1.x - ObstacleCircleRadius, 1.0;
                //          #endif
                //     }
                //     ROS_ERROR("upm_point3D:%f,%f,%f",worldPoint1.z,-worldPoint1.y,worldPoint1.x);

                // }
                // else if (p == LeftMidpoint)
                // {
                    
                //     if (circle_num == 1)
                //         Pc << worldPoint2.z, (worldPoint2.y + ObstacleCircleRadius), worldPoint2.x, 1.0;
                //     else
                //         Pc << worldPoint2.z, (worldPoint2.y + ObstacleCircleRadius), worldPoint2.x, 1.0;
                //     ROS_ERROR("lem_point3D:%f,%f,%f",worldPoint2.z,-(worldPoint2.y + ObstacleCircleRadius),worldPoint2.x);
                // }


                // Pc << worldPoint_upm.z, worldPoint_upm.x, -worldPoint_upm.y -ObstacleCircleRadius, 1.0;
                
                Pc << worldPoint.z, worldPoint.x, -worldPoint.y, 1.0;
                Eigen::Matrix4d Tbc;
                Tbc << 0.0, 0.0, 1.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,

                    1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
                // Tbc <<
                // -1.2011755535951574e-02, -9.9989771278249839e-01,-7.7641291381007780e-03, -1.7537934807651159e-01,
                //  9.9991479583090936e-01, -1.2050922069499759e-02, 5.0176045784532473e-03, -7.4810113937173170e-02,
                // -5.1106562568231781e-03, -7.7031973623564020e-03, 9.9995727005858659e-01,  5.5546220644694155e-01,
                // 0.0, 0.0, 0.0, 1,0;
                // 使用变换矩阵 T 进行坐标变换
                Eigen::Vector4d Pw = Twb * Pc;
                // 提取变换后的世界坐标系下的坐标
                circlePosition.x = Pw(0);
                circlePosition.y = Pw(1);
                circlePosition.z = Pw(2);
                // circlePosition.z = Pw(2) - 0.75;
                // circlePosition.x = point_world.point.x;
                // circlePosition.y = -point_world.point.y;
                // circlePosition.z = point_world.point.z;

                // ROS_ERROR("vins:%f,%f,%f",visual_pose.pose.pose.position.x ,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
                // ROS_ERROR("CircleTruePose:%f,%f,%f",circle_poses_true->poses.front().position.x ,circle_poses_true->poses.front().position.y,circle_poses_true->poses.front().position.z);
                // ROS_ERROR("point3DWorld:%f,%f,%f",circlePosition.x ,circlePosition.y,circlePosition.z);
                return circlePosition;
            }
            else {
                ROS_ERROR("too far!!!");
            }
        }
        return circlePosition;
    #endif
    
}

//************************************
// Description: 根据左右相机中成像坐标求解空间坐标
// Method:    uv2xyz
// FullName:  uv2xyz
// Parameter: Point2f uvLeft
// Parameter: Point2f uvRight
// Returns:   cv::Point3f
//************************************
cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight)
{
	//  [u1]      |X|					  [u2]      |X|
	//Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
	//  [ 1]      |Z|					  [ 1]      |Z|
	//			  |1|								|1|
    //左相机内参数矩阵
    float leftIntrinsic[3][3] ={ 320.0,0,320.0, 0,320.0,240.0, 0,0,1 };
    //左相机畸变系数
    float leftDistortion[1][5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    //左相机旋转矩阵
    float leftRotation[3][3] = {1,0,0, 0,1,0, 0,0,1};
    //左相机平移向量
    float leftTranslation[1][3] = {0, 0, 0};
    //右相机内参数矩阵
    float rightIntrinsic[3][3] ={ 320.0,0,320.0, 0,320.0,240.0, 0,0,1 };
    //右相机畸变系数
    float rightDistortion[1][5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    //右相机旋转矩阵
    float rightRotation[3][3] = {1,0,0, 0,1,0, 0,0,1};
    //右相机平移向量
    // float rightTranslation[1][3] = {-0.095, 0, 0};
    float rightTranslation[1][3] = {-0.096, 0, 0};

    cv::Mat mLeftRotation = cv::Mat(3, 3, CV_32F, leftRotation);
	cv::Mat mLeftTranslation = cv::Mat(3, 1, CV_32F, leftTranslation);
	cv::Mat mLeftRT = cv::Mat(3, 4, CV_32F);//左相机M矩阵
	cv::hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
	cv::Mat mLeftIntrinsic = cv::Mat(3, 3, CV_32F, leftIntrinsic);
	cv::Mat mLeftM = mLeftIntrinsic * mLeftRT;
	//cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;

	cv::Mat mRightRotation = cv::Mat(3, 3, CV_32F, rightRotation);
	cv::Mat mRightTranslation = cv::Mat(3, 1, CV_32F, rightTranslation);
	cv::Mat mRightRT = cv::Mat(3, 4, CV_32F);//右相机M矩阵
	cv::hconcat(mRightRotation, mRightTranslation, mRightRT);
	cv::Mat mRightIntrinsic = cv::Mat(3, 3, CV_32F, rightIntrinsic);
	cv::Mat mRightM = mRightIntrinsic * mRightRT;
	//cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;

	//最小二乘法A矩阵
	cv::Mat A = cv::Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = uvLeft.x * mLeftM.at<float>(2, 0) - mLeftM.at<float>(0, 0);
	A.at<float>(0, 1) = uvLeft.x * mLeftM.at<float>(2, 1) - mLeftM.at<float>(0, 1);
	A.at<float>(0, 2) = uvLeft.x * mLeftM.at<float>(2, 2) - mLeftM.at<float>(0, 2);

	A.at<float>(1, 0) = uvLeft.y * mLeftM.at<float>(2, 0) - mLeftM.at<float>(1, 0);
	A.at<float>(1, 1) = uvLeft.y * mLeftM.at<float>(2, 1) - mLeftM.at<float>(1, 1);
	A.at<float>(1, 2) = uvLeft.y * mLeftM.at<float>(2, 2) - mLeftM.at<float>(1, 2);

	A.at<float>(2, 0) = uvRight.x * mRightM.at<float>(2, 0) - mRightM.at<float>(0, 0);
	A.at<float>(2, 1) = uvRight.x * mRightM.at<float>(2, 1) - mRightM.at<float>(0, 1);
	A.at<float>(2, 2) = uvRight.x * mRightM.at<float>(2, 2) - mRightM.at<float>(0, 2);

	A.at<float>(3, 0) = uvRight.y * mRightM.at<float>(2, 0) - mRightM.at<float>(1, 0);
	A.at<float>(3, 1) = uvRight.y * mRightM.at<float>(2, 1) - mRightM.at<float>(1, 1);
	A.at<float>(3, 2) = uvRight.y * mRightM.at<float>(2, 2) - mRightM.at<float>(1, 2);

	//最小二乘法B矩阵
	cv::Mat B = cv::Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = mLeftM.at<float>(0, 3) - uvLeft.x * mLeftM.at<float>(2, 3);
	B.at<float>(1, 0) = mLeftM.at<float>(1, 3) - uvLeft.y * mLeftM.at<float>(2, 3);
	B.at<float>(2, 0) = mRightM.at<float>(0, 3) - uvRight.x * mRightM.at<float>(2, 3);
	B.at<float>(3, 0) = mRightM.at<float>(1, 3) - uvRight.y * mRightM.at<float>(2, 3);

	cv::Mat XYZ = cv::Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ
	cv::solve(A, B, XYZ, cv::DECOMP_SVD);

	//cout<<"空间坐标为 = "<<endl<<XYZ<<endl;

	//世界坐标系中坐标
	cv::Point3f world;
	world.x = XYZ.at<float>(0, 0);
	world.y = XYZ.at<float>(1, 0);
	world.z = XYZ.at<float>(2, 0);

	return world;
}

//************************************
// Description: 根据ROI区域提取ORB特征并暴力匹配，返回多个匹配点对
// Method:    ORBPointsMathch
// Parameter: 左右目图像、ROI区域vector 左上角x坐标、左上角y坐标、宽度、高度、类名 
// Returns:   std::vector<cv::Point2f> 多对匹配点对
//************************************
std::vector<cv::Point2f> ORBPointsMathch(cv::Mat& image_left, cv::Mat& image_right, std::vector<float>& ROI) {
    // 定义ORB特征提取器
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    // 提取左右图像的ORB特征
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right; //特征点
    cv::Mat descriptors_left, descriptors_right; //描述子
    orb->detectAndCompute(image_left, cv::noArray(), keypoints_left, descriptors_left);
    orb->detectAndCompute(image_right, cv::noArray(), keypoints_right, descriptors_right);

    // 创建用于存储匹配点对的向量
    std::vector<cv::DMatch> matches;
    // 使用暴力匹配器进行特征匹配
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors_left, descriptors_right, matches);

    // 存储匹配点对的坐标
    std::vector<cv::Point2f> matched_points;
    // 提取匹配点对的坐标
    for (size_t i = 0; i < matches.size(); i++) {
        cv::Point2f left_point = keypoints_left[matches[i].queryIdx].pt;
        cv::Point2f right_point = keypoints_right[matches[i].trainIdx].pt;
        // 检查匹配点是否在ROI区域内
        float roi_x = ROI[0];
        float roi_y = ROI[1];
        float roi_width = ROI[2];
        float roi_height = ROI[3];
        if (left_point.x >= roi_x && left_point.x <= roi_x + roi_width &&
            left_point.y >= roi_y && left_point.y <= roi_y + roi_height) {
            matched_points.push_back(left_point);
            ROS_ERROR("left:%d,%d,right:%d,%d",left_point.x,left_point.y,right_point.x,right_point.y);
        }
    }
    // // 显示匹配成功的特征点
    cv::Mat output_image;
    cv::drawMatches(image_left, keypoints_left, image_right, keypoints_right, matches, output_image);

    cv::imshow("Matches", output_image);
    cv::waitKey(0); // 等待按键事件
    return matched_points;
}