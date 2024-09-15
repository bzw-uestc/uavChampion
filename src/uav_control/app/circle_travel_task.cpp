#include "circle_travel_task.hpp"

// void circleTravelTask::inityoloV8(void)
// {
//     std::string sub_type = "s";
//     std::string cuda_post_process = "c";
//     std::string wts_name = "../../yolov8/best.wts";
//     std::string engine_name = "../../yolov8/yolov8s-seg.engine";
//     yolov8.serialize_engine(wts_name, engine_name, sub_type);
//     cuda_preprocess_init(kMaxInputImageSize);
//     yolov8.prepare_buffer(cuda_post_process);
// }

void circleTravelTask::circleTravelMain(void) {
    if(!airsim_reset_flag_) { 
        ROS_ERROR("reseting...");
        airsim_interface_.airsimReset(); //复位无人机位置
        odom_init_start_time_ = ros::Time::now().toSec();
        airsim_reset_flag_ = true;
    }
    else if(!odom_init_flag_) {
        double now = ros::Time::now().toSec();
        double dt = now - odom_init_start_time_;
        if(dt > ODOM_INIT_TIME) {
            odom_init_flag_ = true;
            airsim_interface_.airsimTakeoff(); //无人机起飞
        }
    }
    else if(circle_msg_world_.empty()) { //获取仿真器信息
        circle_msg_ref_ = airsim_interface_.airsimGetCirclePosRef(); //获取障碍圈参考位姿
        circle_msg_true_ = airsim_interface_.airsimGetCirclePosTrue(); //获取障碍圈真实位姿
        // circle_msg_world_.assign(circle_msg_ref_.begin(), circle_msg_ref_.end()); //将参考位姿拷贝 用以后续对比
        // drone_taget_world_.assign(circle_msg_ref_.begin(), circle_msg_ref_.end()); //将参考位姿拷贝 用以后续对比
        circle_msg_world_.assign(circle_msg_true_.begin(), circle_msg_true_.end()); //将参考位姿拷贝 用以后续对比
        drone_taget_world_.assign(circle_msg_true_.begin(), circle_msg_true_.end()); //将参考位姿拷贝 用以后续对比
        mid_points_map_ = getMidPoint(); //获取当前任务下所需的中间点
        std::queue<std::pair<std_msgs::Header,std::vector<cv::Mat>>> queue_empty; //清空队列用
        std::swap(img_detect_buf_,queue_empty); //清空初始化时产生的历史图像
    }
    else { //main
        droneFdbUpdate();  //1.无人机所需的反馈信息更新 里程计等
        // circlePosionWorldUpdate(); //2.感知障碍圈，更新障碍圈的世界坐标
        droneSetGoalPosion(); //3.更新无人机当前帧的目标点
        droneStateUpdate(); //4.根据当前帧的目标障碍圈更新无人机的控制参数
        dronePosionPDControl(); //5.更新无人机PD控制器
    }
}

circleTravelTask::circleTravelTask(ros::NodeHandle &nh): circle_detection_(nh),airsim_interface_(nh){ 
    detect_left_pub_ = nh.advertise<sensor_msgs::Image>("/detect_image_left", 1);
    ego_goal_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal_point", 1);
    circle_pos_world_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/circle_pos_world", 1);
    visual_odom_sub_ = nh.subscribe("/vins_fusion/imu_propagate",1,&circleTravelTask::visualOdometryCallBack,this);
    ego_pos_cmd_sub_ = nh.subscribe("/position_cmd", 10, &circleTravelTask::egoPosCmdCallBack, this); //ego-planner发出的目标位姿
    drone_max_vel_pub_ = nh.advertise<std_msgs::Float64>("/drone_1/max_vel",10); //max vel to pd and ego
    drone_max_acc_pub_ = nh.advertise<std_msgs::Float64>("/drone_1/max_acc",10); //max vel to pd and ego
    drone_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/drone_true_odom",10);
}

void circleTravelTask::droneFdbUpdate(void) {
    // odom update 使用vins作为无人机的参考位姿
    // ROS_ERROR("circle, x:%f,y:%f,z:%f",circle_msg_true_[circle_num_].pos_world.x,circle_msg_true_[circle_num_].pos_world.y,circle_msg_true_[circle_num_].pos_world.z);
    // ROS_ERROR("ego,x:%f,y:%f,z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);
    
    drone_odom_.pose.pose.position.x = -visual_odom_.pose.pose.position.y;
    drone_odom_.pose.pose.position.y = visual_odom_.pose.pose.position.x; //坐标系
    drone_odom_.pose.pose.position.z = visual_odom_.pose.pose.position.z;
    
    drone_odom_.twist.twist.linear.x = -visual_odom_.twist.twist.linear.y;
    drone_odom_.twist.twist.linear.y = visual_odom_.twist.twist.linear.x;
    drone_odom_.twist.twist.linear.z = visual_odom_.twist.twist.linear.z;
    
    drone_odom_.twist.twist.angular.x = visual_odom_.twist.twist.angular.x;
    drone_odom_.twist.twist.angular.y = visual_odom_.twist.twist.angular.y;
    drone_odom_.twist.twist.angular.z = visual_odom_.twist.twist.angular.z;

    drone_odom_.pose.pose.orientation.w = visual_odom_.pose.pose.orientation.w;
    drone_odom_.pose.pose.orientation.x = -visual_odom_.pose.pose.orientation.x;
    drone_odom_.pose.pose.orientation.y = -visual_odom_.pose.pose.orientation.y;
    drone_odom_.pose.pose.orientation.z = -visual_odom_.pose.pose.orientation.z;

    Eigen::Quaterniond quaternion_drone_odom(drone_odom_.pose.pose.orientation.w,drone_odom_.pose.pose.orientation.x,
                                             drone_odom_.pose.pose.orientation.y,drone_odom_.pose.pose.orientation.z);
    Eigen::Matrix3d rotationMatrix = quaternion_drone_odom.toRotationMatrix();
    drone_odom_yaw_ = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    drone_odom_roll_ = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));  // roll: atan2(r32, r33)
    drone_odom_pitch_ = asin(-rotationMatrix(2, 0));
    ROS_ERROR("vins, x:%f,y:%f,z:%f",drone_odom_.pose.pose.position.x,drone_odom_.pose.pose.position.y,drone_odom_.pose.pose.position.z);
    ROS_ERROR("vins, yaw:%f, pitch:%f, roll:%f",drone_odom_yaw_,drone_odom_pitch_,drone_odom_roll_);

    // 使用仿真器真实位姿作为无人机的参考位姿
    // drone_odom_.header.frame_id = "world";
    // drone_odom_.pose.pose.position.x = airsim_interface_.drone_poses_true_->pose.position.x;
    // drone_odom_.pose.pose.position.y = -airsim_interface_.drone_poses_true_->pose.position.y;
    // drone_odom_.pose.pose.position.z = -airsim_interface_.drone_poses_true_->pose.position.z;
    // drone_odom_.pose.pose.orientation = airsim_interface_.drone_poses_true_->pose.orientation;
    
    // drone_odom_pub_.publish(drone_odom_);
}

void circleTravelTask::circlePosionWorldUpdate(void) {
    /////////////////////////////使用Yolo检测器并利用双目相机恢复障碍圈3D坐标/////////////////////////////////////////
    //返回值为：std::vector<std::vector<double>> 外部vector代表该帧图像中有多少个符合检测阈值的障碍圈
    //内部vector为每个障碍圈的信息，大小为6：障碍圈中心xyz三维坐标、长宽的最大值、倾斜度、圈的类型
    if(!img_detect_buf_.empty()) {
        std_msgs::Header img_header = img_detect_buf_.front().first;
        std::vector<cv::Mat> img_detect_vector = img_detect_buf_.front().second;
        cv::Mat img_detect[2];
        img_detect[0] = img_detect_vector[0];
        img_detect[1] = img_detect_vector[1];
        img_detect_buf_.pop();
        std::vector<circleMsg> circle_msg_camera;
        circle_msg_camera = circle_detection_.circleDetectionNewFrame(img_detect,15,250); //20 120为检测阈值 
        sensor_msgs::ImagePtr detect_image_left = cv_bridge::CvImage(img_header, "bgr8", img_detect[0]).toImageMsg();
        detect_left_pub_.publish(detect_image_left); //发布检测到的图像

        // std::string cuda_post_process = "c";
        // // 推理
        // auto start = std::chrono::system_clock::now();
        // cv::Mat mask_ = yolov8.infer(img_detect[0], cuda_post_process, rects);
        // // 位置解算
        // std::vector<Eigen::Matrix3d> ellipse_cone_matrix =  circle_pose.get_Q(mask_, img_detect[0]);
        // // 以Z轴从小到大排序
        // std::vector<Eigen::Vector3d> poses = circle_pose.standardization(ellipse_cone_matrix);

        if(!circle_msg_camera.empty()) { //当前帧下检测到障碍圈
            for(int i = 0; i < circle_msg_camera.size(); i++) {
                //用矩形框计算xyz坐标 z = sqrt((f^2 * S) / A)  x = (u - c_x) * z / fx    y = (v - c_y) * z / fy
                //先进行实体圆环面积的估算、利用倾斜度进行估算
                
                cv::Point3f pos_yolo; //yolo检测框算出的相机坐标系下圈的位置
                // pos_yolo.x = poses[i][0];
                // pos_yolo.y = poses[i][1];
                // pos_yolo.z = poses[i][2];


                double true_square = circle_msg_camera[i].width_max * circle_msg_camera[i].width_max;

                pos_yolo.z = std::sqrt(CAMERA_FX * CAMERA_FY * 1.2 * 1.2 / true_square);
                pos_yolo.x = (circle_msg_camera[i].camera_center.x - CAMERA_CX ) * pos_yolo.z / CAMERA_FX;
                pos_yolo.y = (circle_msg_camera[i].camera_center.y - CAMERA_CY ) * pos_yolo.z / CAMERA_FY;

                
                if(pos_yolo.z < 12.0) { //小于12米的观测才要
                    // // 颜色
                    // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
                    // // 画一个点
                    // cv::circle(img_detect[0], circle_msg_camera_[i].center, 2, color, -1); // 5表示点的半径，-1表示填充点
                    // cv::imshow("detect",img_detect[0]);
                    // cv::waitKey(1);
                    /*获取视觉里程计到世界坐标系的变换矩阵*/
                    Eigen::Quaterniond quaternion(drone_odom_.pose.pose.orientation.w, drone_odom_.pose.pose.orientation.x, drone_odom_.pose.pose.orientation.y, drone_odom_.pose.pose.orientation.z);
                    Eigen::Vector3d translation(drone_odom_.pose.pose.position.x,drone_odom_.pose.pose.position.y,drone_odom_.pose.pose.position.z);
                    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                    Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                    Twb.block<3, 1>(0, 3) = translation;
                    // 创建一个包含原始点坐标的齐次坐标向量
                    Eigen::Vector4d Pc;
                    Pc << pos_yolo.z, -pos_yolo.x, -pos_yolo.y, 1.0;
                    Eigen::Vector4d Pw = Twb * Pc;
                    // 提取变换后的世界坐标系下的坐标
                    cv::Point3f circle_position_world;
                    circle_position_world.x = Pw(0);
                    circle_position_world.y = Pw(1);
                    circle_position_world.z = Pw(2);
                    
                    // ROS_ERROR("stereo, x:%f,y:%f,z:%f",circle_msg_camera_[i].pos.x,circle_msg_camera_[i].pos.y,circle_msg_camera_[i].pos.z);
                    // ROS_ERROR("yolo, x:%f,y:%f,z:%f",pos_yolo.x,pos_yolo.y,pos_yolo.z);
                    // ROS_ERROR("world x:%f,y:%f,z:%f",circle_position_world.pos.x,circle_position_world.pos.y,circle_position_world.pos.z);
                    if(circle_position_world.z < 0.2) circle_position_world.z = 0.2;
                    int closest_circle_num = findClosestCircleNum(circle_position_world); //找到最接近观测的障碍圈
                    if(closest_circle_num == -1) break;

                    float travel_distance = 2.0; //穿越距离，穿越到环后的距离
                    //找到正确的障碍圈位置 更新障碍环世界坐标
                    Eigen::Vector4d Pc_travel;
                    Pc_travel << pos_yolo.z + travel_distance, -pos_yolo.x, -pos_yolo.y, 1.0;
                    Eigen::Vector4d Pw_travel = Twb * Pc_travel;
                    cv::Point3f travel_pos_world; //穿越障碍环 深度+2
                    travel_pos_world.x = Pw_travel(0);
                    travel_pos_world.y = Pw_travel(1);
                    travel_pos_world.z = Pw_travel(2);
                    circleMsg circle_msg_world; //障碍圈世界坐标信息
                    circle_msg_world.pos_world = travel_pos_world;
                    circle_msg_world.pos_camera = pos_yolo;
                    circle_msg_world.width_max = circle_msg_camera[i].width_max;
                    circle_msg_world.ratio = circle_msg_camera[i].ratio;
                    circle_msg_world.type = circle_msg_camera[i].type;
                    circle_msg_world_[closest_circle_num] = circle_msg_world; 

                    //更新障碍环对位坐标
                    circleMsg drone_target_world; //提前对位的中间点
                    if(circle_msg_camera[i].ratio < 0.8 || circle_msg_camera[i].ratio > 1.25 ) {
                        adjust_flag_ = true;
                        drone_target_world = getAdjustmentPoints(circle_position_world, 6);
                    }
                    else { //障碍环正，直接穿越
                        //未对位完成时将目标设为中间点，先让无人机进行对位
                        Eigen::Vector4d Pc_mid;
                        if(pos_yolo.z > 7.0) Pc_mid << pos_yolo.z - 3.0, -pos_yolo.x, -pos_yolo.y, 1.0;
                        else if(pos_yolo.z > 6.0) Pc_mid << pos_yolo.z - 2.0, -pos_yolo.x, -pos_yolo.y, 1.0;
                        else if(pos_yolo.z > 5.0) Pc_mid << pos_yolo.z - 1.0, -pos_yolo.x, -pos_yolo.y, 1.0;
                        else if(pos_yolo.z > 4.0) Pc_mid << pos_yolo.z - 0.0, -pos_yolo.x, -pos_yolo.y, 1.0;
                        else Pc_mid << pos_yolo.z + travel_distance , -pos_yolo.x, -pos_yolo.y, 1.0;
                        Eigen::Vector4d Pw_mid = Twb * Pc_mid;
                        drone_target_world.pos_world.x = Pw_mid(0);
                        drone_target_world.pos_world.y = Pw_mid(1);
                        drone_target_world.pos_world.z = Pw_mid(2);
                        if(drone_target_world.pos_world.z < 0.2) drone_target_world.pos_world.z = 0.2;
                    }
                    drone_taget_world_[closest_circle_num] = drone_target_world;
                }
                else {
                    // ROS_ERROR("too far!");
                }
            }
        }
    }
}

circleMsg circleTravelTask::getAdjustmentPoints(cv::Point3f circleWorldDetect,double offSet){
    double ref_yaw_buf = circle_msg_ref_[circle_num_].yaw;
    Eigen::Quaterniond quaternion_drone_odom(drone_odom_.pose.pose.orientation.w,drone_odom_.pose.pose.orientation.x
                                            ,drone_odom_.pose.pose.orientation.y,drone_odom_.pose.pose.orientation.z);
    Eigen::Matrix3d rotationMatrix = quaternion_drone_odom.toRotationMatrix();
    double drone_yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    if(abs(drone_yaw - ref_yaw_buf) > CV_PI / 2) 
    {
        if(ref_yaw_buf > 0) ref_yaw_buf -= CV_PI;
        else ref_yaw_buf += CV_PI;
    }
    target_pd_yaw = ref_yaw_buf;
    // ROS_ERROR("ref_yaw_buf:%f, drone_yaw:%f",ref_yaw_buf,drone_yaw);
    // 创建一个 Eigen::Quaterniond 对象
    Eigen::Quaterniond quaternion;
    // 设置欧拉角
    quaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(ref_yaw_buf, Eigen::Vector3d::UnitZ());
    // ROS_ERROR("WORLD:%f,%f,%f",circleWorldDetect.x,circleWorldDetect.y,circleWorldDetect.z);
    Eigen::Vector3d translation(circleWorldDetect.x,circleWorldDetect.y,circleWorldDetect.z);
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
    Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    Twb.block<3, 1>(0, 3) = translation;
    
    Eigen::Vector4d Pc;
    // Pc << circle_msg_camera_[i].pos.z, circle_msg_camera_[i].pos.x, -circle_msg_camera_[i].pos.y, 1.0;
    Pc << (-1) * offSet, 0, 0, 1.0;
    Eigen::Vector4d Pw = Twb * Pc;
    // 提取变换后的世界坐标系下的坐标
    circleMsg circle_adjust_position_world;
    circle_adjust_position_world.pos_world.x = Pw(0);
    circle_adjust_position_world.pos_world.y = Pw(1);
    circle_adjust_position_world.pos_world.z = Pw(2);
    if(circle_adjust_position_world.pos_world.z < 0.2) circle_adjust_position_world.pos_world.z = 0.2;
    ROS_ERROR("mid:%f,%f,%f",circle_adjust_position_world.pos_world.x,circle_adjust_position_world.pos_world.y,circle_adjust_position_world.pos_world.z);
    return circle_adjust_position_world;
}

void circleTravelTask::droneSetGoalPosion(void) { //设置无人机目标点
    //每一帧开始先判断无人机是否到达目标障碍圈
    // ROS_ERROR("FLAG:%d",droneReachedLocation(circle_msg_world_[circle_num_].pos_world,drone_odom_,1.0));
    if(droneReachedLocation(circle_msg_world_[circle_num_].pos_world,drone_odom_,0.5)) {
        if(circle_num_ < 16) circle_num_++;
        if(circle_num_ == 8) circle_num_ = 12; //跳过剩余黄圈
        adjust_flag_ = false;
    }
    //寻找是否拥有中间点
    auto mid_point = mid_points_map_.find(circle_num_);
    if(mid_point != mid_points_map_.end()) {
        //判断无人机是否到达中间点
        if(droneReachedLocation(mid_point->second.front(),drone_odom_,2.0)) { //中间点到达条件可放宽一些
            mid_points_map_.erase(mid_point); //删除已到达的中间点
        }
        else {
            //以中间点为目标点
            drone_target_pose_.pose.position.x = mid_point->second.front().x;
            drone_target_pose_.pose.position.y = mid_point->second.front().y;
            drone_target_pose_.pose.position.z = mid_point->second.front().z;
        }
    }
    else { //当前圈没有中间点
        //更新目标点
        drone_target_pose_.pose.position.x = drone_taget_world_[circle_num_].pos_world.x;
        drone_target_pose_.pose.position.y = drone_taget_world_[circle_num_].pos_world.y;
        drone_target_pose_.pose.position.z = drone_taget_world_[circle_num_].pos_world.z;
        //更新圈的位置
        circle_target_pose_.pose.position.x = circle_msg_world_[circle_num_].pos_world.x;
        circle_target_pose_.pose.position.y = circle_msg_world_[circle_num_].pos_world.y;
        circle_target_pose_.pose.position.z = circle_msg_world_[circle_num_].pos_world.z;
    }
    
    //以无人机里程计的姿态角作为目标点的姿态角
    Eigen::Quaterniond quaternion(drone_odom_.pose.pose.orientation.w, drone_odom_.pose.pose.orientation.x, 
    drone_odom_.pose.pose.orientation.y, drone_odom_.pose.pose.orientation.z);
    // Eigen::Quaterniond quaternion = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
    //              * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
    //              * Eigen::AngleAxisd(circle_msg_ref_[circle_num_].yaw , Eigen::Vector3d::UnitZ());
    drone_target_pose_.pose.orientation.w = quaternion.w();
    drone_target_pose_.pose.orientation.x = quaternion.x();
    drone_target_pose_.pose.orientation.y = quaternion.y();
    drone_target_pose_.pose.orientation.z = quaternion.z();

    circle_target_pose_.pose.orientation.w = quaternion.w();
    circle_target_pose_.pose.orientation.x = quaternion.x();
    circle_target_pose_.pose.orientation.y = quaternion.y();
    circle_target_pose_.pose.orientation.z = quaternion.z();

    if(abs(drone_target_pose_.pose.position.x) < 250 && abs(drone_target_pose_.pose.position.y) < 250 && abs(drone_target_pose_.pose.position.z) < 45) {
        drone_target_pose_.header.frame_id = "world";
        ego_goal_point_pub_.publish(drone_target_pose_);  //发布ego-planner规划器目标路点

        circle_target_pose_.header.frame_id = "world";
        circle_pos_world_pub_.publish(circle_target_pose_); 
        // ROS_ERROR("ego,x:%f,y:%f,z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);
        // ROS_ERROR("ego,x:%f,y:%f,z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);
    }
    // ROS_ERROR("x:%f, y:%f, z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);                       
}

//更新无人机状态 根据目标圈的长宽、倾斜度改变无人机的最大速度、控制参数
void circleTravelTask::droneStateUpdate(void){ 
    int threshold1 = 0, threshold2 = 0; //降速的长宽阈值
    enum selectSpeedEnum{FAST,MID,SLOW};
    selectSpeedEnum speed_flag; //速度标志位 0为快速 1为中速 2为慢速

    if(circle_msg_world_[circle_num_].ratio < 0.6 || circle_msg_world_[circle_num_].ratio > 1.6) {
        threshold1 = 25;
        threshold2 = 40;
    }
    else if(circle_msg_world_[circle_num_].ratio < 0.8 || circle_msg_world_[circle_num_].ratio > 1.2) {
        threshold1 = 30;
        threshold2 = 50;
    }
    else {
        threshold1 = 40;
        threshold2 = 70;
    }

    if(circle_msg_world_[circle_num_].pos_camera.x > 1.0 || circle_msg_world_[circle_num_].pos_camera.y > 1.0 ) {
        //该情况表明xy距离很远，需要慢速对位
        speed_flag = SLOW;
    }
    else if(circle_msg_world_[circle_num_].width_max > threshold2) { //降到低速模式
        speed_flag = SLOW;
    }
    else if(circle_msg_world_[circle_num_].width_max > threshold1) { //中速模式
        speed_flag = MID;
    }
    else { //高速模式
        speed_flag = FAST;
    }

    float drone_max_vel,drone_max_acc;
    if(speed_flag == SLOW) { //降到低速模式
        drone_max_vel = MAX_VEL_SLOW;
        drone_max_acc = MAX_ACC_NORMAL;
    }
    else if(speed_flag == MID) { //中速模式
        drone_max_vel = MAX_VEL_MID;
        drone_max_acc = MAX_ACC_NORMAL;
    }
    else { //高速模式
        drone_max_vel = MAX_VEL_FAST;
        drone_max_acc = MAX_ACC_FAST;
    }

    std_msgs::Float64 drone_max_vel_msgs;
    drone_max_vel_msgs.data = drone_max_vel;
    drone_max_vel_pub_.publish(drone_max_vel_msgs);
    std_msgs::Float64 drone_max_acc_msgs;
    drone_max_acc_msgs.data = drone_max_acc;
    drone_max_acc_pub_.publish(drone_max_acc_msgs);
}

void circleTravelTask::dronePosionPDControl(void) { //无人机位置PD控制 
    // if(ego_init_flag_) { //ego初始化完成后，调用PD控制器
    //     airsim_interface_.airsimSetGoalPosition(ego_pos_cmd_.position.x,ego_pos_cmd_.position.y,
    //                                             ego_pos_cmd_.position.z,ego_pos_cmd_.yaw);
    //     if (adjust_flag_ == true) {
    //        airsim_interface_.airsimSetGoalPosition(ego_pos_cmd_.position.x,ego_pos_cmd_.position.y,
    //                                                ego_pos_cmd_.position.z,target_pd_yaw);
    //     }
    // }

    
    // 不使用ego-planner 直接用目标值飞
    // double dx = drone_target_pose_.pose.position.x - drone_odom_.pose.pose.position.x;
    // double dy = drone_target_pose_.pose.position.y - drone_odom_.pose.pose.position.y;
    // double dz = drone_target_pose_.pose.position.z - drone_odom_.pose.pose.position.z;
    // double posecmd_yaw = std::atan2(dy, dx);
    // // 调用 PD控制器控制无人机运动
    // airsim_interface_.airsimSetGoalPosition(drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,
    //                                         drone_target_pose_.pose.position.z,posecmd_yaw);

    static ros::Time last_time;
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time;
    last_time = current_time;
    double dt = period.toSec() ;  //大约是0.01s 100Hz

    static quadrotor_msgs::PositionCommand last_ego_cmd;
    if(ego_init_flag_) {
        // SE3-Control
        
        // ROS_ERROR("ego,x:%f,y:%f,z:%f,yaw:%f",ego_pos_cmd_.position.x,ego_pos_cmd_.position.y,ego_pos_cmd_.position.z,ego_pos_cmd_.yaw);
        Desired_State_t ego_desired_state;
        ego_desired_state.p.x() = -ego_pos_cmd_.position.y;
        ego_desired_state.p.y() =  ego_pos_cmd_.position.x;
        ego_desired_state.p.z() =  ego_pos_cmd_.position.z;

        ego_desired_state.v.x() = -ego_pos_cmd_.velocity.y;
        ego_desired_state.v.y() =  ego_pos_cmd_.velocity.x;
        ego_desired_state.v.z() =  ego_pos_cmd_.velocity.z;

        ego_desired_state.a.x() = -ego_pos_cmd_.acceleration.y;
        ego_desired_state.a.y() =  ego_pos_cmd_.acceleration.x;
        ego_desired_state.a.z() =  ego_pos_cmd_.acceleration.z;

        ego_desired_state.jerk.x() = -(ego_pos_cmd_.acceleration.y - last_ego_cmd.acceleration.y) / dt;
        ego_desired_state.jerk.y() =  (ego_pos_cmd_.acceleration.x - last_ego_cmd.acceleration.x) / dt;
        ego_desired_state.jerk.z() =  (ego_pos_cmd_.acceleration.z - last_ego_cmd.acceleration.z) / dt;
        // ROS_ERROR("ego_pos:   x:%f,  y:%f,  z:%f ",ego_desired_state.p.x(),ego_desired_state.p.y(),ego_desired_state.p.z());
        // ROS_ERROR("vel:   x:%f,  y:%f,  z:%f ",ego_desired_state.v.x(),ego_desired_state.v.y(),ego_desired_state.v.z());
        // ROS_ERROR("acc:   x:%f,  y:%f,  z:%f ",ego_desired_state.a.x(),ego_desired_state.a.y(),ego_desired_state.a.z());
        // ROS_ERROR("jerk:   x:%f,  y:%f,  z:%f ",ego_desired_state.jerk.x(),ego_desired_state.jerk.y(),ego_desired_state.jerk.z());
        ego_desired_state.yaw = -ego_pos_cmd_.yaw;
        ego_desired_state.head_rate = -ego_pos_cmd_.yaw_dot;
        // ROS_ERROR("ego_yaw:%f, yaw_rate:%f",ego_desired_state.yaw,ego_desired_state.head_rate);
        Odom_Data_t odom;
        odom.p.x() = drone_odom_.pose.pose.position.x;
        odom.p.y() = drone_odom_.pose.pose.position.y;
        odom.p.z() = drone_odom_.pose.pose.position.z;

        odom.v.x() = drone_odom_.twist.twist.linear.x;
        odom.v.y() = drone_odom_.twist.twist.linear.y;
        odom.v.z() = drone_odom_.twist.twist.linear.z;
        // ROS_ERROR("odom_pos:   x:%f,  y:%f,  z:%f ",odom.p.x(),odom.p.y(),odom.p.z());
        // ROS_ERROR("odom_vel:   x:%f,  y:%f,  z:%f ",odom.v.x(),odom.v.y(),odom.v.z());
        odom.q.x() = drone_odom_.pose.pose.orientation.x;
        odom.q.y() = drone_odom_.pose.pose.orientation.y;
        odom.q.z() = drone_odom_.pose.pose.orientation.z;
        odom.q.w() = drone_odom_.pose.pose.orientation.w;

        odom.w.x() = drone_odom_.twist.twist.angular.x;
        odom.w.y() = drone_odom_.twist.twist.angular.y;
        odom.w.z() = drone_odom_.twist.twist.angular.z;

        // ROS_ERROR("yaw:  target:%f,  odom:%f",ego_desired_state.yaw,drone_odom_yaw_);
        // ego_odom.w.x() = 1.0f;
        // ego_odom.w.y() = 1.0f;
        // ego_odom.w.z() = 1.0f;
        // ROS_ERROR("odom_v:   x:%f,  y:%f,  z:%f ",ego_odom.v.x(),ego_odom.v.y(),ego_odom.v.z());
        // ROS_ERROR("odom_w:   x:%f,  y:%f,  z:%f ",ego_odom.w.x(),ego_odom.w.y(),ego_odom.w.z());
        Controller_Output_t se3_output;
        se3_controller_.update(ego_desired_state,odom,se3_output);
        Eigen::Vector3d angle_rate;
        angle_rate.x() = se3_output.roll_rate;
        angle_rate.y() = se3_output.pitch_rate; 
        angle_rate.z() = se3_output.yaw_rate;
        ROS_ERROR("thrust:%f,  yaw_rate:%f,  pitch_rate:%f,  roll_rate:%f",se3_output.thrust,angle_rate.z(),angle_rate.y(),angle_rate.x());
        airsim_interface_.airsimAngleRateThrottleCtrl(se3_output.thrust,angle_rate);
        // angle_rate.x() =  0.0f;  // roll正， 向右飞
        // angle_rate.y() =  0.3f;  // pitch正，抬头，向后飞
        // angle_rate.z() =  0.0f;  // yaw正，  顺时针飞
        // airsim_interface_.airsimAngleRateThrottleCtrl(0.6,angle_rate);

        // Eigen::Vector3d zero = Eigen::Vector3d::Zero();
        // airsim_interface_.airsimAngleRateThrottleCtrl(0.6,zero);
        last_ego_cmd = ego_pos_cmd_;
    }
    
}

int circleTravelTask::findClosestCircleNum(cv::Point3f circle_pos) {         
    int closest_index = -1; // 初始化为无效值
    double minx_distance = 7.0; // 初始化为大于4米的值
    double miny_distance = 7.0;
    double minz_distance = 5.0;

    for (int i = circle_num_; i < circle_msg_ref_.size(); i++) {
        double xDistance = std::abs(circle_msg_ref_[i].pos_world.x - circle_pos.x);
        double yDistance = std::abs(circle_msg_ref_[i].pos_world.y - circle_pos.y);
        double zDistance = std::abs(circle_msg_ref_[i].pos_world.z - circle_pos.z);

        if (xDistance < minx_distance && yDistance < miny_distance && zDistance < minz_distance) {
            minx_distance = xDistance;
            miny_distance = yDistance;
            minz_distance = zDistance;
            closest_index = i;
        }
    }
    return closest_index;
}

//获取中间点，起飞前调用
std::map<int,std::vector<cv::Point3f>> circleTravelTask::getMidPoint(void) {
    //清空中间点，避免重复叠加
    std::map<int,std::vector<cv::Point3f>> mid_points_map;
    int circle_num;
    std::vector<cv::Point3f> circle_mid_points;
    cv::Point3f mid_point;
    circle_num = 4;
    mid_point.x = 105;
    mid_point.y = -35;
    mid_point.z = 5;
    circle_mid_points.emplace_back(mid_point);
    mid_points_map.emplace(circle_num,circle_mid_points);

    circle_num = 12;
    circle_mid_points.clear();
    mid_point.x = circle_msg_ref_[circle_num].pos_world.x + 25;
    mid_point.y = circle_msg_ref_[circle_num].pos_world.y;
    mid_point.z = circle_msg_ref_[circle_num].pos_world.z + 3;
    circle_mid_points.emplace_back(mid_point);
    mid_points_map.emplace(circle_num,circle_mid_points);

    if(!circle_msg_ref_.empty()) { //以下中间点需要用到圈的参考位姿
        circle_num = 13;
        circle_mid_points.clear();
        mid_point.x = circle_msg_ref_[circle_num].pos_world.x;
        mid_point.y = circle_msg_ref_[circle_num].pos_world.y + 20;
        mid_point.z = circle_msg_ref_[circle_num].pos_world.z;
        circle_mid_points.emplace_back(mid_point);
        mid_points_map.emplace(circle_num,circle_mid_points);

        circle_num = 15;
        circle_mid_points.clear();
        mid_point.x = circle_msg_ref_[circle_num - 1].pos_world.x + 15;
        mid_point.y = circle_msg_ref_[circle_num].pos_world.y - 10;
        mid_point.z = circle_msg_ref_[circle_num].pos_world.z;
        circle_mid_points.emplace_back(mid_point);
        mid_point.x = circle_msg_ref_[circle_num].pos_world.x - 10;
        mid_point.y = circle_msg_ref_[circle_num].pos_world.y - 8;
        mid_point.z = circle_msg_ref_[circle_num].pos_world.z;
        circle_mid_points.emplace_back(mid_point);
        mid_points_map.emplace(circle_num,circle_mid_points);
    }
    return mid_points_map;
}

void circleTravelTask::visualOdometryCallBack(const nav_msgs::Odometry& drone_vins_odom){
    visual_odom_ = drone_vins_odom;
}

void circleTravelTask::egoPosCmdCallBack(const quadrotor_msgs::PositionCommand& ego_pos_cmd) {
    ego_pos_cmd_ = ego_pos_cmd;
    ego_init_flag_ = true;
}


// bool circleTravelTask::uav_reached_location(geometry_msgs::PoseStamped ref,nav_msgs::Odometry fdb,double distance_dxyz) {
//     double dx,dy,dz;
//     dx = abs(ref.pose.position.x - fdb.pose.pose.position.x );
//     dy = abs(-ref.pose.position.y - fdb.pose.pose.position.y );
//     dz = abs(-ref.pose.position.z - fdb.pose.pose.position.z );
//     if(dx < distance_dxyz && dy < distance_dxyz && dz < distance_dxyz) {
//         return true;
//     }
//     return false;
// }

bool circleTravelTask::droneReachedLocation(cv::Point3f ref,nav_msgs::Odometry fdb,double distance_dxyz) {
    double dx,dy,dz;
    dx = abs(ref.x - fdb.pose.pose.position.x );
    dy = abs(ref.y - fdb.pose.pose.position.y );
    dz = abs(ref.z - fdb.pose.pose.position.z );
    if(dx < distance_dxyz && dy < distance_dxyz && dz < distance_dxyz) {
        return true;
    }
    return false;
}