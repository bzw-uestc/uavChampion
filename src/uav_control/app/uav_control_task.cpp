#include "uav_control_task.hpp"

std::chrono::_V2::system_clock::time_point last_time;
std::chrono::_V2::system_clock::time_point last_time2;
// kalmanFilter kf_camera_x(1, 10);
// kalmanFilter kf_camera_y(1, 10);
// kalmanFilter kf_camera_z(1, 20);

void uavControl::uavControlTask(void) {
    if(!reset_flag) { //!reset_flag
        sim_reset_client.call(sim_reset);   
        reset_flag = 1;
        odom_init_start_time = ros::Time::now().toSec();
    }
    else if(!odom_init_flag) {
        double now = ros::Time::now().toSec();
        double dt = now - odom_init_start_time;
        if(dt > ODOM_INIT_TIME) odom_init_flag=1;
    }
    else if(!takeoff_flag) {
        sim_takeoff.request.waitOnLastTask = 1;
        // takeoff_flag = sim_takeoff_client.call(sim_takeoff);
        takeoff_flag = true;
        ego_target_pose.pose.position.x = 5 ;
        ego_target_pose.pose.position.y = 0 ;
        ego_target_pose.pose.position.z = 1 ;
    }
    else if(circle_msg_flag) { 
        cv::Point3f circlePositionWorld;
        cv::Point3f circlePositionCamera; 
        if(!circle_detect_msg_queue.empty()) { //当检测信息不为空时且在靠近目标点一定范围内尝试恢复检测目标的3D坐标 
            auto start = std::chrono::system_clock::now();
            int time = std::chrono::duration_cast<std::chrono::milliseconds>(start - last_time).count();
            // ROS_ERROR("task2:%dms",time);
            last_time = start;
            circle_detect_msg = circle_detect_msg_queue.front();
            // ROS_ERROR("queue_size:%d",circle_detect_msg_queue.size());
            circle_detect_msg_queue.pop();
            //////////////////////////////先计算最大面积的检测框 判断是否靠近障碍圈////////////////////////////////////////////////
            int circleNum[2] = {0,0}, circleTag[2] = {0,0}; //检测圈的个数、最大面积对应的是第几个障碍圈
            double circleSquareMax[2] = {0,0}; //检测圈的最大面积
            double circleWidthMax[2] = {0,0};
            float circleTypeDetect[2] = {-1,-1};
            for(int i=0; i < circle_detect_msg.size(); i++) { //遍历双目的检测信息 计算面积
                circleNum[i] = circle_detect_msg[i].size() / 5; //检测结果存放的信息为：左上角x坐标、左上角y坐标、宽度、高度、类别名
                for(int j=0; j<circleNum[i]; j++) {
                    double squareTemp = circle_detect_msg[i][j*5+2] * circle_detect_msg[i][j*5+3];
                    int type = circle_detect_msg[i][j*5+4];
                    double widthTemp = std::max(circle_detect_msg[i][j*5+2] , circle_detect_msg[i][j*5+3]);
                    
                    if(widthTemp > circleWidthMax[i]) {
                        circleWidthMax[i] = widthTemp;
                        circleTag[i] = j;  //标记第几个圈是最大面积圈
                    }
                    if(squareTemp > circleSquareMax[i]) {
                        circleSquareMax[i] = squareTemp;
                        circleTypeDetect[i] = type;
                        circleTag[i] = j;  //标记第几个圈是最大面积圈
                    }
                }
            }

            double aspectRatioLeft = circle_detect_msg[0][circleTag[0]*5+2] / circle_detect_msg[0][circleTag[0]*5+3];
            int ratio_falg = 0;
            ROS_ERROR("aspectRatioLeft:%f",aspectRatioLeft);
            if(aspectRatioLeft < 0.6 || aspectRatioLeft > 1.6) ratio_falg = -2;
            else if(aspectRatioLeft > 0.8 && aspectRatioLeft < 1.2) ratio_falg =1;
            else ratio_falg = -1;
            // ROS_ERROR("ratio:%d",ratio_falg);
            // circleTypeDetect
            
            // if(!uav_reached_location(ego_target_pose,visual_pose,15.0) ) {
            //     drone_slowly_flag = 0;
            // }
            // else if(circleSquareMax[0] > 3000.0 && circleSquareMax[1] > 3000.0 && uav_reached_location(ego_target_pose,visual_pose,12.0) && drone_slowly_flag <=2) {
            //     drone_slowly_flag = 2;
            // }
            // else if(circleSquareMax[0] > 1200.0 && circleSquareMax[1] > 1200.0 && uav_reached_location(ego_target_pose,visual_pose,12.0) && drone_slowly_flag <= 1 ) { //&& circleTypeDetect[0] == circle_type && circleTypeDetect[1] == circle_type
            //     drone_slowly_flag = 1;
            // }
            // else if(uav_reached_location(ego_target_pose,visual_pose,8.0) && drone_slowly_flag<=1) {
            //     drone_slowly_flag = 1;
            // }

            double threshold1,threshold2;
            if(ratio_falg == -1) {
                threshold1 = 25;
                threshold2 = 50;
            }
            else if(ratio_falg == -2) {
                threshold1 = 20;
                threshold2 = 40;
            }
            else {
                threshold1 = 35;
                threshold2 = 70;
            }
            if(!uav_reached_location(ego_target_pose,visual_pose,15.0) ) {
                drone_slowly_flag = 0;
            }
            else if(circleWidthMax[0] > threshold2 && circleWidthMax[1] > threshold2 && uav_reached_location(ego_target_pose,visual_pose,12.0) && drone_slowly_flag <=2) {
                drone_slowly_flag = 2;
            }
            else if(circleWidthMax[0] > threshold1 && circleWidthMax[1] > threshold1 && uav_reached_location(ego_target_pose,visual_pose,12.0) && drone_slowly_flag <= 1 ) { //&& circleTypeDetect[0] == circle_type && circleTypeDetect[1] == circle_type
                drone_slowly_flag = 1;
            }
            else if(uav_reached_location(ego_target_pose,visual_pose,8.0) && drone_slowly_flag<=1) {
                drone_slowly_flag = 1;
            }

            // else{
            //     drone_slowly_flag = 0;
            // }
            if(circle_num == 13 && !drone_slowly_flag) drone_slowly_flag = 1;
            if(circle_num == 14 && !drone_slowly_flag) drone_slowly_flag = 1;
            if(circle_num == 15 && circle15_flag && circle15_flag2 && !drone_slowly_flag) drone_slowly_flag = 2;  //////!!!!!!!!!! 
            // ROS_ERROR("ego: x:%f,y:%f,z:%f",ego_target_pose.pose.position.x,ego_target_pose.pose.position.y,ego_target_pose.pose.position.z);
            // ROS_ERROR("visual: x:%f,y:%f,z:%f",visual_pose.pose.pose.position.x,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
            //////////////////////////////当检测面积足够大且无人机离参考位姿较近时才解算圈的世界坐标////////////////////////////////////
            if(circle_num == 16) {  
                cv::Point3f circle16_ref_cruuent;
                circle16_ref_cruuent.x = circle_poses_ref->poses.at(circle_num).position.x;
                circle16_ref_cruuent.y = circle_poses_ref->poses.at(circle_num).position.y;
                circle16_ref_cruuent.z = -circle_poses_ref->poses.at(circle_num).position.z;
                
                if(circle16_min_z > circle16_ref_cruuent.z){
                    circle16_min_z = circle16_ref_cruuent.z;
                }
                if(circle16_ref_cruuent.y - circle16_ref_last.y > 0.0)
                {
                    circle16_rotation_flag = 1;                  //向右旋转
                }
                else if (circle16_ref_cruuent.y - circle16_ref_last.y < 0.0)
                {
                    circle16_rotation_flag = -1;                // 向左旋转
                }
                circle16_ref_last = circle16_ref_cruuent;
                // ROS_ERROR("circle16_rotation_flag:%d;circle16_min_z:%f",circle16_rotation_flag,circle16_min_z);
                

                if(circle16_obs_flag && !circle16_arrive_flag &&uav_reached_location(ego_target_pose,visual_pose,0.5)) {
                    circle16_arrive_flag = true;
                    circle16_arrive_time = ros::Time::now().toSec();
                    ROS_ERROR("get circle16 mid point!");
                }
                else if(circle16_arrive_flag && !circle16_clash_flag && circleSquareMax[0] > 8000.0 && circleSquareMax[1] > 8000.0 && circleSquareMax[0] < 100000.0 && circleSquareMax[1] < 100000.0) {
                    // ROS_ERROR("suare:%f",circleSquareMax[0]);
                    // std::vector<double> upm = detectCirclePosion(UpperMidpoint,circleTag);
                    std::vector<double> lom = detectCirclePosion(LowerMidpoint,circleTag);
                    // std::vector<double> lem = detectCirclePosion(LeftMidpoint,circleTag);
                    // std::vector<double> rim = detectCirclePosion(RightMidpoint,circleTag); //获取检测信息
                    std::vector<cv::Point2f> circleImagePoints = 
                    {cv::Point2f(lom[0],lom[1])}; //图像的像素点坐标 pnp用
                    // // 颜色
                    // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
                    // // 画一个点
                    // cv::circle(image_left, circleImagePoints[0], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[1], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[2], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[3], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // // 显示图像
                    // cv::imshow("result_left", image_left);
                    // cv::imshow("result_right", image_right);
                    // cv::waitKey(1);
                    if (lom[2] != 0) {//筛除异常值 会把深度异常的点筛除掉
                        circlePositionCamera.x = lom[2];
                        circlePositionCamera.y = lom[3];
                        circlePositionCamera.z = lom[4];
                        circlePositionCamera.z += 15;
                        Eigen::Quaterniond quaternion(visual_pose.pose.pose.orientation.w, visual_pose.pose.pose.orientation.x, visual_pose.pose.pose.orientation.y, visual_pose.pose.pose.orientation.z);
                        Eigen::Vector3d translation(visual_pose.pose.pose.position.x, visual_pose.pose.pose.position.y, -visual_pose.pose.pose.position.z);
                        Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                        Eigen::Vector4d Pc;
                        Eigen::Vector4d Pw;
                        
                        /*获取视觉里程计到世界坐标系的变换矩阵*/
                        Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                        Twb.block<3, 1>(0, 3) = translation;
                        // 创建一个包含原始点坐标的齐次坐标向量

                        Pc << circlePositionCamera.z, circlePositionCamera.x, -circlePositionCamera.y, 1.0;
                        Pw = Twb * Pc;
                        // 提取变换后的世界坐标系下的坐标
                        circlePositionWorld.x = Pw(0);
                        circlePositionWorld.y = Pw(1);
                        circlePositionWorld.z = Pw(2);
                        if(circlePositionWorld.z < 0.2) circlePositionWorld.z = 0.2;

                        visual_detect_flag = true;
                        double circle16_dt = ros::Time::now().toSec() - circle16_arrive_time;
                        
                        if(circle16_dt > 1 && circle16_dt < 4 && circlePositionWorld.z < 10.0 ) {
                            circle16_world_y_sum += circlePositionWorld.y;
                            circle16_world_y_cnt++;
                            circle16_world_y_avr = circle16_world_y_sum / circle16_world_y_cnt;
                            ROS_ERROR("now:%f,avr:%f",circlePositionWorld.y,circle16_world_y_avr);
                            if((circlePositionWorld.z - 0.5) > circle16_clash_point.z) {
                                if(circle16_clash_point.y > circle16_world_y_avr){
                                    circle16_clash_rotation = 1;
                                }
                                else if(circle16_clash_point.y < circle16_world_y_avr)
                                {
                                    circle16_clash_rotation = -1;
                                }
                                
                                if(circle16_clash_point.y == 0 || abs(circle16_clash_point.y - circlePositionWorld.y) < 2.0) {
                                    circle16_clash_point.x = circlePositionWorld.x;
                                    circle16_clash_point.y = circlePositionWorld.y;
                                    circle16_clash_point.z = circlePositionWorld.z - 0.5;

                                    Eigen::Vector4d Pc_mid;
                                    Pc_mid << 0, circlePositionCamera.x, -circlePositionCamera.y, 1.0;
                                    Eigen::Vector4d Pw_mid = Twb * Pc_mid;
                                    // 提取变换后的世界坐标系下的坐标

                                    circle16_aim_point.x = Pw_mid(0);
                                    circle16_aim_point.y = Pw_mid(1);
                                    circle16_aim_point.z = Pw_mid(2) - 0.5;
                                    ROS_ERROR("clash:x:%f y:%f z:%f ",circle16_clash_point.x,circle16_clash_point.y,circle16_clash_point.z);
                                    ROS_ERROR("aim:x:%f y:%f z:%f ",circle16_aim_point.x,circle16_aim_point.y,circle16_aim_point.z);
                                }
                                // ROS_ERROR("circlePositionWorld.y:%f;visual_pose.pose.pose.position.y:%f",circlePositionWorld.y,visual_pose.pose.pose.position.y);
                            }
                        }
                        if(circle16_dt > 4) {  //
                            circle16_aim_flag = true;
                        }
                    
                        ROS_ERROR("circle16_clash_rotation:%d,circle_rotation_true:%d",circle16_clash_rotation,circle16_rotation_flag);
                        if(circle16_dt > 7 && circle16_ref_cruuent.z - circle16_min_z < 0.4 && circle16_rotation_flag == circle16_clash_rotation) {  //
                            circle16_clash_flag = true;
                            circle16_point = 1;
                            // if(circle16_clash_rotation == 1) { //冲右上点
                            //     circle16_clash_point.y -= 1;
                            // }
                            // else { //冲左上点
                            //     circle16_clash_point.y += 1;
                            // }
                        }
                        // ROS_ERROR("circle:x:%f,y:%f,z:%f,maxy:%f,maxX:%f,minX:%f",circlePositionCamera.x,circlePositionCamera.y,circlePositionCamera.z,circle16_camera_y_max,circle16_camera_x_max,circle16_camera_x_min);
                    }
                }
                else if(circleSquareMax[0] > 1500.0 && circleSquareMax[1] > 1500.0 && circleSquareMax[0] < 2000.0 && circleSquareMax[1] < 2000.0 && !circle16_obs_flag) {
                    std::vector<double> upm = detectCirclePosion(UpperMidpoint,circleTag);
                    std::vector<double> lom = detectCirclePosion(LowerMidpoint,circleTag);
                    std::vector<double> lem = detectCirclePosion(LeftMidpoint,circleTag);
                    std::vector<double> rim = detectCirclePosion(RightMidpoint,circleTag); //获取检测信息
                    std::vector<cv::Point2f> circleImagePoints = 
                    {cv::Point2f(upm[0],upm[1]), cv::Point2f(lom[0],lom[1]), cv::Point2f(lem[0],lem[1]), cv::Point2f(rim[0],rim[1])}; //图像的像素点坐标 pnp用

                    if (upm[2] != 0 && lom[2] != 0 && lem[2] != 0 && rim[2] != 0) {//筛除异常值 会把深度异常的点筛除掉
                        circlePositionCamera.x = (upm[2] + lom[2] + lem[2] + rim[2]) / 4;
                        circlePositionCamera.y = (upm[3] + lom[3] + lem[3] + rim[3]) / 4;
                        circlePositionCamera.z = (upm[4] + lom[4] + lem[4] + rim[4]) / 4;

                        Eigen::Quaterniond quaternion(visual_pose.pose.pose.orientation.w, visual_pose.pose.pose.orientation.x, visual_pose.pose.pose.orientation.y, visual_pose.pose.pose.orientation.z);
                        Eigen::Vector3d translation(visual_pose.pose.pose.position.x, visual_pose.pose.pose.position.y, -visual_pose.pose.pose.position.z);
                        Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                        Eigen::Vector4d Pc;
                        Eigen::Vector4d Pw;
                        if(circlePositionCamera.z > 6.0) {
                            circlePositionCamera.z -= 4.0;
                            /*获取视觉里程计到世界坐标系的变换矩阵*/
                            Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                            Twb.block<3, 1>(0, 3) = translation;
                            // 创建一个包含原始点坐标的齐次坐标向量

                            Pc << circlePositionCamera.z, circlePositionCamera.x, -circlePositionCamera.y, 1.0;
                            Pw = Twb * Pc;
                            // 提取变换后的世界坐标系下的坐标
                            circlePositionWorld.x = Pw(0);
                            circlePositionWorld.y = Pw(1);
                            circlePositionWorld.z = Pw(2);
                            if(circlePositionWorld.z < 0.2) circlePositionWorld.z = 0.2;

                            // while(!circle16PositionWorld.empty())
                            // {
                            //     circle16PositionWorld.pop_back();
                            // }
                            // circle16PositionWorld.push_back(circlePositionWorld);
                            // circlePositionCamera.z = circlePositionCamera.z + 14;
                            // Pc << circlePositionCamera.z, circlePositionCamera.x, -circlePositionCamera.y, 1.0;
                            // Pw = Twb * Pc;
                            // circlePositionWorld.x = Pw(0);
                            // circlePositionWorld.y = Pw(1);
                            // circlePositionWorld.z = Pw(2) + 0.3;
                            // circle16PositionWorld.push_back(circlePositionWorld);
                            
                            visual_detect_flag = true;
                        }
                    }
                }
                else if(circleSquareMax[0] > 3100.0 && circleSquareMax[1] > 3100.0 && !circle16_obs_flag) {
                    circle16_obs_flag = true;
                    nav_msgs::Odometry visual_pose_temp;
                    visual_pose_temp = visual_pose;
                    Eigen::Quaterniond q1(visual_pose_temp.pose.pose.orientation.w, visual_pose_temp.pose.pose.orientation.x, 
                    visual_pose_temp.pose.pose.orientation.y, visual_pose_temp.pose.pose.orientation.z);
                    Eigen::Matrix3d rotationMatrix = q1.toRotationMatrix();
                    Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(0, 1, 2);
                    circle16_yaw = 0;
                }
            }
            // else if (circleSquareMax[0] > 1200.0 && circleSquareMax[1] > 1200.0 && circleSquareMax[0] < 20000.0 && circleSquareMax[1] < 20000.0 threshold1
            // && uav_reached_location(ego_target_pose,visual_pose,14.0) ){ //&& circleTypeDetect[0] == circle_type && circleTypeDetect[1] == circle_type
            else if (circleWidthMax[0] > threshold1 && circleWidthMax[1] > threshold1 && circleSquareMax[0] < 16000.0 && circleSquareMax[1] < 16000.0 
            && uav_reached_location(ego_target_pose,visual_pose,14.0) && circleTypeDetect[0] == circle_type && circleTypeDetect[1] == circle_type){ //
                std::vector<double> upm = detectCirclePosion(UpperMidpoint,circleTag);
                std::vector<double> lom = detectCirclePosion(LowerMidpoint,circleTag);
                std::vector<double> lem = detectCirclePosion(LeftMidpoint,circleTag);
                std::vector<double> rim = detectCirclePosion(RightMidpoint,circleTag); //获取检测信息
                std::vector<cv::Point2f> circleImagePoints = 
                {cv::Point2f(upm[0],upm[1]), cv::Point2f(lom[0],lom[1]), cv::Point2f(lem[0],lem[1]), cv::Point2f(rim[0],rim[1])}; //图像的像素点坐标 pnp用

                if (upm[2] != 0 && lom[2] != 0 && lem[2] != 0 && rim[2] != 0) {//筛除异常值 会把深度异常的点筛除掉
                    circlePositionCamera.x = (upm[2] + lom[2] + lem[2] + rim[2]) / 4;
                    circlePositionCamera.y = (upm[3] + lom[3] + lem[3] + rim[3]) / 4;
                    circlePositionCamera.z = (upm[4] + lom[4] + lem[4] + rim[4]) / 4;
                    // // circlePositionCamera.y -= 0.3;
                    // // 颜色
                    // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
                    // // 画一个点
                    // cv::circle(image_left, circleImagePoints[0], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[1], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[2], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::circle(image_left, circleImagePoints[3], 2, color, -1); // 5表示点的半径，-1表示填充点
                    // // 显示图像
                    // cv::imshow("result_left", image_left);
                    // // cv::imshow("result_right", image_right);
                    // cv::waitKey(1);
                    if(circlePositionCamera.z > 5.0) {
                        // if(!kal_init_flag) {
                        //     ROS_ERROR("init kal_filter");
                        //     kal_init_flag = true;
                        //     kf_camera_x.kalmanSetInit(circlePositionCamera.x);
                        //     kf_camera_y.kalmanSetInit(circlePositionCamera.y);
                        //     kf_camera_z.kalmanSetInit(circlePositionCamera.z);
                        // }
                        // else {
                        //     double kf_x = kf_camera_x.update(circlePositionCamera.x);
                        //     double kf_y = kf_camera_y.update(circlePositionCamera.y);
                        //     double kf_z = kf_camera_z.update(circlePositionCamera.z);
                        //     // ROS_ERROR("cam:x:%f,y:%f,z:%f,kal:x:%f,y:%f,z:%f",circlePositionCamera.x,circlePositionCamera.y,circlePositionCamera.z,kf_x,kf_y,kf_z);
                        //     circlePositionCamera.x = kf_x;
                        //     circlePositionCamera.y = kf_y;
                        //     circlePositionCamera.z = kf_z;
                        // }
                        if((abs(circlePositionCamera.x) > 2.5 || abs(circlePositionCamera.y) > 2.5 ) && circlePositionCamera.z > 7.0) circlePositionCamera.z -= 5.0;
                        else if((abs(circlePositionCamera.x) > 2.5 || abs(circlePositionCamera.y) > 2.5 ) && circlePositionCamera.z > 6.0) circlePositionCamera.z -= 4.0;
                        else if((abs(circlePositionCamera.x) > 2.5 || abs(circlePositionCamera.y) > 2.5 ) && circlePositionCamera.z > 5.0) circlePositionCamera.z -= 3.0;
                        
                        if((abs(circlePositionCamera.x) > 2.5 || abs(circlePositionCamera.y) > 2.5 )) {
                            aim_flag = 1;
                            drone_slowly_flag = 3;
                        }
                        else {
                            aim_flag = 0;
                        }
                        if( drone_slowly_flag == 3 && ratio_falg == 1 && abs(circlePositionCamera.x) < 1.0 && abs(circlePositionCamera.y) < 1.0) {
                            drone_slowly_flag = 2;
                        }
                        // if(!uav_reached_location(ego_target_pose,visual_pose,8.0)) {
                        //     drone_slowly_flag = 2;
                        // }
                        #ifdef TRUE_POSE_DEBUGE
                            Eigen::Quaterniond quaternion(drone_poses_true->pose.orientation.w, drone_poses_true->pose.orientation.x, drone_poses_true->pose.orientation.y, drone_poses_true->pose.orientation.z);
                            Eigen::Vector3d translation(drone_poses_true->pose.position.x, drone_poses_true->pose.position.y, -drone_poses_true->pose.position.z);
                        #else
                        /*获取视觉里程计到世界坐标系的变换矩阵*/
                            Eigen::Quaterniond quaternion(visual_pose.pose.pose.orientation.w, visual_pose.pose.pose.orientation.x, visual_pose.pose.pose.orientation.y, visual_pose.pose.pose.orientation.z);
                            Eigen::Vector3d translation(visual_pose.pose.pose.position.x, visual_pose.pose.pose.position.y, -visual_pose.pose.pose.position.z);
                        #endif
                        Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                        Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                        Twb.block<3, 1>(0, 3) = translation;
                        // 创建一个包含原始点坐标的齐次坐标向量
                        Eigen::Vector4d Pc;
                        Pc << circlePositionCamera.z, circlePositionCamera.x, -circlePositionCamera.y, 1.0;
                        Eigen::Vector4d Pw = Twb * Pc;
                        // 提取变换后的世界坐标系下的坐标
                        circlePositionWorld.x = Pw(0);
                        circlePositionWorld.y = Pw(1);
                        circlePositionWorld.z = Pw(2);
                        if(circlePositionWorld.z < 0.2) circlePositionWorld.z = 0.2;
                        visual_detect_flag = true;
                    }
                    // ROS_ERROR("x:%f,y:%f,z:%f",circlePositionWorld.x,circlePositionWorld.y,circlePositionWorld.z);
                }
            }
            else {
                aim_flag = false;
            }
        }

        auto start2 = std::chrono::system_clock::now();
        int time2 = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - last_time2).count();
        // ROS_ERROR("task3:%dms",time2);
        last_time2 = start2;
        //////////////////////////////判断是否到达目标点，前往下一个障碍圈////////////////////////////////////////
        if(uav_reached_location(ego_target_pose,visual_pose,1.5) && circle_num!=16)  
        {
            if(pd_delay_flag == false) {
                pd_delay_flag = true;
                pd_delay_start_time = ros::Time::now().toSec();
            }
            if(circle_num <= 15) {
                circle_num += 1;
                ROS_ERROR("circle_num:%d",circle_num);
                ROS_ERROR("visual:x:%f,y:%f,z:%f",visual_pose.pose.pose.position.x,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
                ROS_ERROR("circle:x:%f,y:%f,z:%f",circle_poses_ref->poses.at(circle_num).position.x,circle_poses_ref->poses.at(circle_num).position.y,circle_poses_ref->poses.at(circle_num).position.z);
                if(uav_reached_location(circle_poses_ref->poses.at(circle_num).position.x,circle_poses_ref->poses.at(circle_num).position.y,
                circle_poses_ref->poses.at(circle_num).position.z,visual_pose,8.0)) {
                    circle_num += 1;
                    ROS_ERROR("jump to circle:%d",circle_num);
                }
            }
            // if(uav_reached_location(circle_poses_ref->poses.at(circle_num).position.x,))
            if(circle_num == 8) { //跳过剩下的黄圈
                circle_num = 12;
            }

            if(circle_num == 4) {
                ego_target_pose.pose.position.x = 105 ;
                ego_target_pose.pose.position.y = -35 ;
                ego_target_pose.pose.position.z = 5 ;
            }
            else if(circle_num == 12) {
                ego_target_pose.pose.position.x = -10 ;
                ego_target_pose.pose.position.y = -50 ;
                ego_target_pose.pose.position.z = 3 ;
            }
            else if(circle_num == 13) {
                ego_target_pose.pose.position.x =  circle_poses_ref->poses.at(circle_num).position.x -6;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y- 10);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 15) {
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x-15;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y+10);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
    
            if(circle_num >= 12) circle_type = 1; //只识别红圈
            else if(circle_num >= 5) circle_type = 0;  //只识别黄圈
            else circle_type = 1;
           
            visual_detect_flag = false;  
            drone_slowly_flag = 0;
            mid_point_flag = false;    //重置状态机
            visual_detect_flag = false;
        }


        // tf::Quaternion visulQuatcirclePoint0;
        // tf::quaternionMsgToTF(visual_pose.pose.pose.orientation, visulQuatcirclePoint0);
        // double visulQuatcirclePoint0roll, visulQuatcirclePoint0pitch, visulQuatcirclePoint0yaw;//定义存储r\p\y的容器
        // tf::Matrix3x3(visulQuatcirclePoint0).getRPY(visulQuatcirclePoint0roll, visulQuatcirclePoint0pitch, visulQuatcirclePoint0yaw);//进行转换
        // ROS_ERROR("circle16_yaw:%f;visulQuatcirclePoint0yaw:%f",circle16_yaw,visulQuatcirclePoint0yaw);

        // ROS_ERROR("visual_pose:%f,%f,%f",visual_pose.pose.pose.position.x,visual_pose.pose.pose.position.y,visual_pose.pose.pose.position.z);
        // ROS_ERROR("circle16_aim_point:%f,%f,%f",circle16_aim_point.x,circle16_aim_point.y,circle16_aim_point.z);
        // ROS_ERROR("circle16_clash_point:%f,%f,%f",circle16_clash_point.x,circle16_clash_point.y,circle16_aim_point.z);
  
        if(abs(visual_pose.pose.pose.position.x - circle16_aim_point.x) < 0.4 && abs(visual_pose.pose.pose.position.y - circle16_aim_point.y) < 0.4 
               && abs((-1) * visual_pose.pose.pose.position.z - circle16_aim_point.z) < 0.4 && circle_num==16 && circle16_point == 0 && circle16_clash_flag)  
        {
            circle16_point = 1;
            // ROS_ERROR("TESTcircle16_point = 1;");
        }
        else{
            if(!circle16_loop_flag)
            {
                if(abs(visual_pose.pose.pose.position.x - circle16_clash_point.x) < 0.4 && abs(visual_pose.pose.pose.position.y - circle16_clash_point.y) < 0.4 
                  && abs((-1) * visual_pose.pose.pose.position.z - circle16_aim_point.z) < 0.4 && circle_num==16 && circle16_point == 1 && circle16_clash_flag)
                {
                    circle16_loop_flag = true;
                    circle16_point = 0;
                    // ROS_ERROR("TESTcircle16_point = 0;");
                }
            }
            else if(abs(visual_pose.pose.pose.position.x - (circle16_clash_point.x - 10)) < 0.4 && abs(visual_pose.pose.pose.position.y - circle16_clash_point.y) < 0.4 
               && abs((-1) * visual_pose.pose.pose.position.z - circle16_aim_point.z) < 0.4 && circle_num==16 && circle16_point == 1 && circle16_clash_flag)
            {
                circle16_loop_flag = true;
                circle16_point = 0;
                // ROS_ERROR("TESTcircle16_point = 0;");
            }
        }
        // ROS_ERROR("circle16_point:%d",circle16_point);
        // ROS_ERROR("circle16_loop_flag:%d",circle16_loop_flag);
        ////////////////////////////按策略重新规划下一帧的目标点////////////////////////////////////////////////
        if(circle_num == 16 && circle16_arrive_flag) {
            if(circlePositionWorld.x!=0 && circlePositionWorld.y!=0 && circlePositionWorld.z!=0) {
                ego_target_pose.pose.position.x = circle16_aim_point.x;
                ego_target_pose.pose.position.y = -circle16_aim_point.y;
                ego_target_pose.pose.position.z = circle16_aim_point.z;
                nav_msgs::Odometry visual_pose_temp;
                visual_pose_temp = visual_pose;

                Eigen::Quaterniond q1(visual_pose_temp.pose.pose.orientation.w, visual_pose_temp.pose.pose.orientation.x, 
                visual_pose_temp.pose.pose.orientation.y, visual_pose_temp.pose.pose.orientation.z);
                Eigen::Matrix3d rotationMatrix = q1.toRotationMatrix();
                Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(0, 1, 2);
                euler_angles[2] = -euler_angles[2];
                Eigen::Matrix3d modifiedRotationMatrix;
                modifiedRotationMatrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond q2(modifiedRotationMatrix);
                ego_target_pose.pose.orientation.w = q2.w();
                ego_target_pose.pose.orientation.x = q2.x();
                ego_target_pose.pose.orientation.y = q2.y();
                ego_target_pose.pose.orientation.z = q2.z();
                            
            }
        }
        else if(visual_detect_flag) { //进入视觉模式 && (circle_num != 13 || (circle_num == 13 && circle13_flag)）
            if(circlePositionWorld.x!=0 && circlePositionWorld.y!=0 && circlePositionWorld.z!=0) {
                ego_target_pose.pose.position.x = circlePositionWorld.x;
                ego_target_pose.pose.position.y = -circlePositionWorld.y;
                ego_target_pose.pose.position.z = circlePositionWorld.z;
                // ROS_ERROR("World:x:%f,y:%f,z:%f",ego_target_pose.pose.position.x,ego_target_pose.pose.position.y,ego_target_pose.pose.position.z);
                nav_msgs::Odometry visual_pose_temp;
                visual_pose_temp = visual_pose;

                Eigen::Quaterniond q1(visual_pose_temp.pose.pose.orientation.w, visual_pose_temp.pose.pose.orientation.x, 
                visual_pose_temp.pose.pose.orientation.y, visual_pose_temp.pose.pose.orientation.z);
                Eigen::Matrix3d rotationMatrix = q1.toRotationMatrix();
                Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(0, 1, 2);
                euler_angles[2] = -euler_angles[2];
                Eigen::Matrix3d modifiedRotationMatrix;
                modifiedRotationMatrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond q2(modifiedRotationMatrix);
                ego_target_pose.pose.orientation.w = q2.w();
                ego_target_pose.pose.orientation.x = q2.x();
                ego_target_pose.pose.orientation.y = q2.y();
                ego_target_pose.pose.orientation.z = q2.z();            
            }
        }
        else if (!circle16_loop_flag){
            if(circle_num == 15 && circle15_flag && circle15_flag2) {  //发布正常点
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
                ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            }
            else if(circle_num == 15 && !circle15_flag && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num-1).position.x+15;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y+10);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 15 && !circle15_flag && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle15_flag = true;
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x-10;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y+8);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
                ROS_ERROR("get mid point, cirlce_num:%d",circle_num);
            }
            else if(circle_num == 15 && !circle15_flag2 && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x-10;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y+8);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 15 && !circle15_flag2 && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle15_flag2 = true;
                ROS_ERROR("get mid point2, cirlce_num:%d",circle_num);
            }

            else if(circle_num == 13 && circle13_flag) {  //发布正常点
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
                ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            }
            else if(circle_num == 13 && !circle13_flag && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x =  circle_poses_ref->poses.at(circle_num).position.x -6;
                ego_target_pose.pose.position.y = -(circle_poses_ref->poses.at(circle_num).position.y-10);
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 13 && !circle13_flag && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle13_flag = true;
                ROS_ERROR("get mid point, cirlce_num:%d",circle_num);
            }

            else if(circle_num == 12 && circle12_flag) {  //发布正常点
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
                ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            }
            else if(circle_num == 12 && !circle12_flag && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x = -10 ;
                ego_target_pose.pose.position.y = -50 ;
                ego_target_pose.pose.position.z =  3 ;
            }
            else if(circle_num == 12 && !circle12_flag && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle12_flag = true;
                ROS_ERROR("get mid point, cirlce_num:%d",circle_num);
            }

            else if(circle_num == 4 && circle4_flag && circle4_flag2) {  //发布正常点
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
                ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            }
            else if(circle_num == 4 && !circle4_flag && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x = 105 ;
                ego_target_pose.pose.position.y = -35 ;
                ego_target_pose.pose.position.z = 5 ;
            }
            else if(circle_num == 4 && !circle4_flag && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle4_flag = true;
                ROS_ERROR("get mid point, cirlce_num:%d",circle_num);
                ego_target_pose.pose.position.x = 80;
                ego_target_pose.pose.position.y = -45;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 4 && !circle4_flag2 && !uav_reached_location(ego_target_pose,visual_pose,5)) { //发布中间点 
                ego_target_pose.pose.position.x = 80;
                ego_target_pose.pose.position.y = -45;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z);
            }
            else if(circle_num == 4 && !circle4_flag2 && uav_reached_location(ego_target_pose,visual_pose,5)) { //已到达中间点
                circle4_flag2 = true;
                ROS_ERROR("get mid point2, cirlce_num:%d",circle_num);
            }
            else {
                ego_target_pose.pose.position.x = circle_poses_ref->poses.at(circle_num).position.x ;
                ego_target_pose.pose.position.y = -circle_poses_ref->poses.at(circle_num).position.y ;
                ego_target_pose.pose.position.z = abs(circle_poses_ref->poses.at(circle_num).position.z) ;
            }
            double circle_roll = 0, circle_pitch = 0, circle_yaw = circle_poses_ref->poses.at(circle_num).yaw;

            Eigen::Matrix3d rotationMatrix;
            rotationMatrix = Eigen::AngleAxisd(circle_roll, Eigen::Vector3d::UnitX())
                   * Eigen::AngleAxisd(circle_pitch, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(circle_yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond circle_q(rotationMatrix);
            
            ego_target_pose.pose.orientation.w = circle_q.w();
            ego_target_pose.pose.orientation.x = circle_q.x();
            ego_target_pose.pose.orientation.y = circle_q.y();
            ego_target_pose.pose.orientation.z = circle_q.z();
        }
        
        if(circle_num!= 0 &&!uav_reached_location(ego_target_pose,visual_pose,18.0) ) {
            drone_slowly_flag = -1;
        }

        if(circle_num == 15 && !circle15_flag) {
            drone_max_vel = 10;
            drone_max_acc = 20;
        }
        else if(circle_num == 15 && !circle15_flag2) {
            drone_max_vel = MAX_VEL_FAST;
            drone_max_acc = 10;
        }
        else if(circle16_clash_flag) {
            drone_max_vel = 10;
            drone_max_acc = MAX_ACC_FAST;
        }
        else if(drone_slowly_flag == -1) {
            drone_max_vel = MAX_VEL_FAST;
            drone_max_acc = MAX_ACC_FAST_FAST;
        }
        else if(drone_slowly_flag == 3) {
            drone_max_vel = MAX_VEL_SLOW_SLOW;
            drone_max_acc = MAX_ACC_NORMAL;
        }
        else if(drone_slowly_flag == 2) {
            drone_max_vel = MAX_VEL_SLOW;
            drone_max_acc = MAX_ACC_NORMAL;
        }
        else if(drone_slowly_flag == 1) {
            drone_max_vel = MAX_VEL_MID;
            drone_max_acc = MAX_ACC_NORMAL;
        }
        else {
            if(circle_num == 0) {
                drone_max_vel = 5.0;
                drone_max_acc = 12;
            }
            drone_max_vel = MAX_VEL_FAST;
            drone_max_acc = MAX_ACC_FAST;
        }

        if(abs(ego_target_pose.pose.position.x) < 250 && abs(ego_target_pose.pose.position.y) < 250 && abs(ego_target_pose.pose.position.z) < 45) {
            ego_target_pose.header.frame_id = "world";
            ego_goal_point_pub.publish(ego_target_pose);  
            std_msgs::Float64 drone_max_vel_msgs;
            drone_max_vel_msgs.data = drone_max_vel;
            drone_max_vel_pub.publish(drone_max_vel_msgs);
            std_msgs::Float64 drone_max_acc_msgs;
            drone_max_acc_msgs.data = drone_max_acc;
            drone_max_acc_pub.publish(drone_max_acc_msgs);
            // ROS_ERROR("max_vel%f",drone_max_vel);
            // ROS_ERROR("circle_num:%d,egox:%f,egoy:%f,egoz:%f",circle_num,ego_target_pose.pose.position.x,ego_target_pose.pose.position.y,ego_target_pose.pose.position.z);
        }
        
        
        if(pd_delay_flag)
        {
            double now = ros::Time::now().toSec();
            double dt = now - pd_delay_start_time;
            if(dt > PD_DELAY_TIME) {
                pd_delay_flag = false;
            }
        }
        else if(circle_num == 16 && circle16_clash_flag && !circle16_loop_flag) {
            posCmd.request.x = circle16_clash_point.x;
            posCmd.request.y = circle16_clash_point.y;
            posCmd.request.z = -circle16_aim_point.z;
            setGoalPosition_client.call(posCmd);
        }
        else if(circle_num == 16 && circle16_aim_flag && !circle16_loop_flag) {
            posCmd.request.x = circle16_aim_point.x;
            posCmd.request.y = circle16_aim_point.y;
            posCmd.request.z = -circle16_aim_point.z;
            setGoalPosition_client.call(posCmd);
        }
        else if(circle_num == 16 && circle16_arrive_flag && !circle16_loop_flag) {

        }

        else if(circle_num == 16 && circle16_obs_flag && !circle16_loop_flag) {
            posCmd.request.x = ego_target_pose.pose.position.x;
            posCmd.request.y = -ego_target_pose.pose.position.y;
            posCmd.request.z = -ego_target_pose.pose.position.z;
            posCmd.request.yaw = circle16_yaw;
            setGoalPosition_client.call(posCmd);
        }
        else if(circle_num == 16 && visual_detect_flag == false) {
            posCmd.request.x = ego_target_pose.pose.position.x;
            posCmd.request.y = -ego_target_pose.pose.position.y;
            posCmd.request.z = -ego_target_pose.pose.position.z;
            posCmd.request.yaw = 0;
            setGoalPosition_client.call(posCmd);
        }
        else if(ego_init_flag && !circle16_loop_flag) {
                posCmd.request.x = posture_cmd.x;
                posCmd.request.y = -posture_cmd.y;
                posCmd.request.z = -posture_cmd.z;
                // if(!aim_flag) {  //too far no yaw
                    posCmd.request.yaw = -posture_cmd.yaw;
                // }
                setGoalPosition_client.call(posCmd);
        }
        else if(circle16_loop_flag && circle_num == 16 && circle16_point == 0){
            posCmd.request.x = circle16_aim_point.x;
            posCmd.request.y = circle16_aim_point.y;
            posCmd.request.z = -circle16_aim_point.z;
            setGoalPosition_client.call(posCmd);
            ROS_ERROR("TEST3");
        }
        else if(circle16_loop_flag && circle_num == 16 && circle16_point == 1){
            posCmd.request.x = circle16_clash_point.x - 10;
            posCmd.request.y = circle16_clash_point.y;
            posCmd.request.z = -circle16_aim_point.z;
            setGoalPosition_client.call(posCmd);
            ROS_ERROR("TEST4");
        }
    }
}

uavControl::uavControl(ros::NodeHandle &nh) {
    posCmd.request.x = 0;
    posCmd.request.y = 0;
    posCmd.request.z = 0;
    posCmd.request.yaw = 0;
    
    circle_poses_ref_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, &uavControl::circlePosesRef_callBack,this); //仿真器输出的障碍圈参考位姿
    circle_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, &uavControl::circlePosesTrue_callBack,this); //仿真器输出的障碍圈真实位姿
    tree_poses_true_sub = nh.subscribe("/airsim_node/drone_1/debug/tree_poses_gt", 10, &uavControl::treePosesTrue_callBack,this); //仿真器输出的树的位姿
    ego_pos_cmd_sub = nh.subscribe("/position_cmd", 10, &uavControl::egoPosCmd_callBack, this); //ego-planner发出的目标位姿
    // ego_path_sub = nh.subscribe("/position_cmd", 10, &uavControl::egoPosCmd_callBack, this); //ego-planner发出的目标位姿
    drone_true_odom_sub = nh.subscribe("/airsim_node/drone_1/debug/pose_gt",1,&uavControl::dronePosesTrue_callBack,this);
    // visual_odom_sub = nh.subscribe("/vins_fusion/imu_propagate",1,&uavControl::droneVisualPose_callBack,this);
    visual_odom_sub = nh.subscribe("/vins_fusion/imu_propagate_for_pd",1,&uavControl::droneVisualPose_callBack,this);
    pd_vel_sub = nh.subscribe("/airsim_node/drone_1/vel_cmd_body_frame_yaw",1,&uavControl::pdVelPose_callBack,this);
    
    drone_vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);  //pd-vel-controler
    drone_pose_pub = nh.advertise<airsim_ros::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 10); //pd-location-controler
    drone_max_vel_pub = nh.advertise<std_msgs::Float64>("/drone_1/max_vel",10); //max vel to pd and ego
    drone_max_acc_pub = nh.advertise<std_msgs::Float64>("/drone_1/max_acc",10); //max vel to pd and ego

    ego_goal_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal_point", 1); 

    setGoalPosition_client = nh.serviceClient<airsim_ros::SetLocalPosition>("/airsim_node/local_position_goal/override");
    sim_takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    sim_reset_client = nh.serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    
    // int circle_num = 4;
    // geometry_msgs::PoseStamped mid_point;
    // mid_point.pose.position.x = 90;
    // mid_point.pose.position.y = 30;
    // mid_point.pose.position.z = 5;
    // mid_point_map.emplace(circle_num,mid_point);
    // std::cout << "Init Finsh!!!" << std::endl;
}

void uavControl::uavSetGoalPostion(void) {

}

void uavControl::dronePosesTrue_callBack(const geometry_msgs::PoseStampedConstPtr& drone_poses) {
    drone_pose_true_flag = true;
    // ROS_ERROR("recive_circle");
    drone_poses_true = drone_poses;
    
    // ROS_ERROR("TRUE:%f,%f,%f, %f", drone_poses_true->orientation.w,drone_poses_true->orientation.x,drone_poses_true->orientation.y,drone_poses_true->orientation.z);
}

void uavControl::circlePosesRef_callBack(const airsim_ros::CirclePosesConstPtr& circle_pose) {
    circle_poses_ref = circle_pose;
    circle_msg_flag = true;
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
    posture_cmd.x = pos_cmds.position.x;
    posture_cmd.y = pos_cmds.position.y;
    posture_cmd.z = pos_cmds.position.z;
    posture_cmd.yaw = pos_cmds.yaw;
    ego_init_flag = true;
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

std::vector<double> uavControl::detectCirclePosion(SelectPoint p, int *circleTag)
{
    std::vector<double> circleDetectMsg; //存储检测结果变量
    // ORBPointsMathch(image_left,image_right,circle_detect_msg[0]);  //该方法先不用 等我回来调试
    int x1_left, y1_left, x1_right, y1_right;
    if ( p == UpperMidpoint) {
        x1_left = circle_detect_msg[0][circleTag[0]*5] + 0.5 *circle_detect_msg[0][circleTag[0]*5+2]; //左上角x坐标+宽度的一半
        y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 3; //左上角y坐标
        x1_right = circle_detect_msg[1][circleTag[1]*5] + 0.5 *circle_detect_msg[1][circleTag[1]*5+2];
        y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 3;
    }
    else if (p == LowerMidpoint) {
        x1_left = circle_detect_msg[0][circleTag[0]*5] + 0.5 *circle_detect_msg[0][circleTag[0]*5+2]; //左上角x坐标+宽度的一半
        y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + circle_detect_msg[0][circleTag[0]*5+3] - 3; //左下角y坐标
        x1_right = circle_detect_msg[1][circleTag[1]*5] + 0.5 *circle_detect_msg[1][circleTag[1]*5+2];
        y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + circle_detect_msg[1][circleTag[1]*5+3] - 3;
        if (circle_num == 16){
            x1_left = circle_detect_msg[0][circleTag[0]*5] + 0.5 *circle_detect_msg[0][circleTag[0]*5+2]; //左上角x坐标+宽度的一半
            y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + circle_detect_msg[0][circleTag[0]*5+3] - 12; //左下角y坐标
            x1_right = circle_detect_msg[1][circleTag[1]*5] + 0.5 *circle_detect_msg[1][circleTag[1]*5+2];
            y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + circle_detect_msg[1][circleTag[1]*5+3] - 12;
        }
    }
    else if (p == LeftMidpoint) {
        x1_left = circle_detect_msg[0][circleTag[0]*5] + 3; //左上角x坐标
        y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 0.5 * circle_detect_msg[0][circleTag[0]*5+3]; //左上角y坐标 - 高度一半
        x1_right = circle_detect_msg[1][circleTag[1]*5] + 3;
        y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 0.5 * circle_detect_msg[1][circleTag[1]*5+3] ;
    }
    else if (p == RightMidpoint) {
        x1_left = circle_detect_msg[0][circleTag[0]*5] + circle_detect_msg[0][circleTag[0]*5+2] - 3; //左上角x坐标+宽度
        y1_left = circle_detect_msg[0][circleTag[0]*5 + 1] + 0.5 * circle_detect_msg[0][circleTag[0]*5+3] ; 
        x1_right = circle_detect_msg[1][circleTag[1]*5] + circle_detect_msg[1][circleTag[1]*5+2] - 3;
        y1_right = circle_detect_msg[1][circleTag[1]*5 + 1] + 0.5 * circle_detect_msg[1][circleTag[1]*5+3];
    }
    cv::Point2f left(x1_left , y1_left);
    cv::Point2f right(x1_right, y1_right);
    cv::Point3f circlePostionCamera = uv2xyz(left, right); //相机坐标系下圈的位姿
    circleDetectMsg.push_back(x1_left);
    circleDetectMsg.push_back(y1_left);
    circleDetectMsg.push_back(circlePostionCamera.x);
    circleDetectMsg.push_back(circlePostionCamera.y);
    circleDetectMsg.push_back(circlePostionCamera.z);
    
    if(circlePostionCamera.x <12.0 && circlePostionCamera.y <12.0 && circlePostionCamera.z <12.0) {
        // return worldPoint;
        return circleDetectMsg;
    }
    else {
        // ROS_ERROR("too far!!!");
        std::vector<double> zero(5,0);
        return zero;
    }
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
	world.z = XYZ.at<float>(2, 0) + 1.0; //穿过圆环 距离+1.0

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

bool uavControl::uav_reached_location(geometry_msgs::PoseStamped ref,nav_msgs::Odometry fdb,double distance_dxyz) {
    double dx,dy,dz;
    dx = abs(ref.pose.position.x - fdb.pose.pose.position.x );
    dy = abs(-ref.pose.position.y - fdb.pose.pose.position.y );
    dz = abs(-ref.pose.position.z - fdb.pose.pose.position.z );
    if(dx < distance_dxyz && dy < distance_dxyz && dz < distance_dxyz) {
        return true;
    }
    return false;
}

bool uavControl::uav_reached_location(double target_x,double target_y,double target_z,nav_msgs::Odometry fdb,double distance_dxyz) {
    double dx,dy,dz;
    dx = abs(target_x - fdb.pose.pose.position.x );
    dy = abs(target_y - fdb.pose.pose.position.y );
    dz = abs(target_z - fdb.pose.pose.position.z );
    if(dx < distance_dxyz && dy < distance_dxyz && dz < distance_dxyz) {
        return true;
    }
    return false;
}