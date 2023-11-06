#include "circle_travel_task.hpp"


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
        circle_msg_world_.assign(circle_msg_ref_.begin(), circle_msg_ref_.end()); //将参考位姿拷贝 用以后续对比
        mid_point_map_ = getMidPoint(); //获取当前任务下所需的中间点
        std::queue<std::pair<std_msgs::Header,std::vector<cv::Mat>>> queue_empty; //清空队列用
        std::swap(img_detect_buf_,queue_empty); //清空初始化时产生的历史图像
    }
    else { //main
        droneFdbUpdate();  //1.无人机所需的反馈信息更新 里程计等
        circlePosionWorldUpdate(); //2.感知障碍圈，更新障碍圈的世界坐标
        droneSetGoalPosion(); //3.更新无人机当前帧的目标点
        droneStateUpdate(); //4.根据当前帧的目标障碍圈更新无人机的控制参数
        dronePosionPDControl(); //5.更新无人机PD控制器
    }
}

circleTravelTask::circleTravelTask(ros::NodeHandle &nh): circle_detection_(nh),airsim_interface_(nh){ 
    detect_left_pub_ = nh.advertise<sensor_msgs::Image>("/detect_image_left", 1);
    ego_goal_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal_point", 1); 
    visual_odom_sub_ = nh.subscribe("/vins_fusion/imu_propagate_for_pd",1,&circleTravelTask::visualOdometryCallBack,this);
    ego_pos_cmd_sub_ = nh.subscribe("/position_cmd", 10, &circleTravelTask::egoPosCmdCallBack, this); //ego-planner发出的目标位姿
    drone_max_vel_pub_ = nh.advertise<std_msgs::Float64>("/drone_1/max_vel",10); //max vel to pd and ego
    drone_max_acc_pub_ = nh.advertise<std_msgs::Float64>("/drone_1/max_acc",10); //max vel to pd and ego
}

void circleTravelTask::droneFdbUpdate(void) {
    //odom update
    // ROS_ERROR("circle, x:%f,y:%f,z:%f",circle_msg_true_[circle_num_].pos.x,circle_msg_true_[circle_num_].pos.y,circle_msg_true_[circle_num_].pos.z);
    // ROS_ERROR("ego,x:%f,y:%f,z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);
    // ROS_ERROR("drone, x:%f,y:%f,z:%f",drone_odom_.pose.pose.position.x,drone_odom_.pose.pose.position.y,drone_odom_.pose.pose.position.z);
    drone_odom_.pose.pose.position.x = visual_odom_.pose.pose.position.x;
    drone_odom_.pose.pose.position.y = -visual_odom_.pose.pose.position.y;
    drone_odom_.pose.pose.position.z = -visual_odom_.pose.pose.position.z;
    drone_odom_.pose.pose.orientation = visual_odom_.pose.pose.orientation;
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
        circle_msg_camera_.clear();
        circle_msg_camera_ = circle_detection_.circleDetectionNewFrame(img_detect,40,150); //20 120为检测阈值 
        sensor_msgs::ImagePtr detect_image_left = cv_bridge::CvImage(img_header, "bgr8", img_detect[0]).toImageMsg();
        detect_left_pub_.publish(detect_image_left); //发布检测到的图像
        if(!circle_msg_camera_.empty()) { //当前帧下检测到障碍圈
            for(int i = 0; i < circle_msg_camera_.size(); i++) {
                //用矩形框计算xyz坐标 z = sqrt((f^2 * S) / A)  x = (u - c_x) * z / fx    y = (v - c_y) * z / fy
                double yolo_z = std::sqrt(CAMERA_FX*CAMERA_FY * 1.2 * 1.2 / circle_msg_camera_[i].square);
                double yolo_x = (circle_msg_camera_[i].center.x - CAMERA_CX ) * yolo_z / CAMERA_FX;
                double yolo_y = (circle_msg_camera_[i].center.y - CAMERA_CY ) * yolo_z / CAMERA_FY;
                
                // // 颜色
                // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
                // // 画一个点
                // cv::circle(img_detect[0], circle_msg_camera_[i].center, 2, color, -1); // 5表示点的半径，-1表示填充点
                // cv::imshow("detect",img_detect[0]);
                // cv::waitKey(1);
                /*获取视觉里程计到世界坐标系的变换矩阵*/
                Eigen::Quaterniond quaternion(drone_odom_.pose.pose.orientation.w, drone_odom_.pose.pose.orientation.x, drone_odom_.pose.pose.orientation.y, drone_odom_.pose.pose.orientation.z);
                Eigen::Vector3d translation(drone_odom_.pose.pose.position.x,-drone_odom_.pose.pose.position.y,drone_odom_.pose.pose.position.z);
                Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity(); //IMU坐标系到世界坐标系的变换 实际上就是视觉里程计
                Twb.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
                Twb.block<3, 1>(0, 3) = translation;
                // 创建一个包含原始点坐标的齐次坐标向量
                Eigen::Vector4d Pc;
                // Pc << circle_msg_camera_[i].pos.z, circle_msg_camera_[i].pos.x, -circle_msg_camera_[i].pos.y, 1.0;
                Pc << yolo_z, yolo_x, -yolo_y, 1.0;
                Eigen::Vector4d Pw = Twb * Pc;
                // 提取变换后的世界坐标系下的坐标
                circleMsg circle_position_world;
                circle_position_world.pos.x = Pw(0);
                circle_position_world.pos.y = -Pw(1);
                circle_position_world.pos.z = Pw(2);
                
                // ROS_ERROR("stereo, x:%f,y:%f,z:%f",circle_msg_camera_[i].pos.x,circle_msg_camera_[i].pos.y,circle_msg_camera_[i].pos.z);
                ROS_ERROR("yolo, x:%f,y:%f,z:%f",yolo_x,yolo_y,yolo_z);
                circle_position_world.width_max = circle_msg_camera_[i].width_max;
                circle_position_world.ratio = circle_msg_camera_[i].ratio;
                circle_position_world.type = circle_msg_camera_[i].type;
                // ROS_ERROR("world x:%f,y:%f,z:%f",circle_position_world.pos.x,circle_position_world.pos.y,circle_position_world.pos.z);
                if(circle_position_world.pos.z < 0.2) circle_position_world.pos.z = 0.2;
                int closest_circle_num = findClosestCircleNum(circle_position_world); //找到最接近观测的障碍圈
                if(closest_circle_num == -1) break;
                //未对位完成时将目标设为中间点，先让无人机进行对位
                Eigen::Vector4d Pc_mid;
                if(yolo_z > 7.0) Pc_mid << yolo_z - 3.0, yolo_x, -yolo_y, 1.0;
                else if(yolo_z > 6.0) Pc_mid << yolo_z - 2.0, yolo_x, -yolo_y, 1.0;
                else if(yolo_z > 5.0) Pc_mid << yolo_z - 1.0, yolo_x, -yolo_y, 1.0;
                else if(yolo_z > 4.0) Pc_mid << yolo_z - 0.0, yolo_x, -yolo_y, 1.0;
                else Pc_mid << yolo_z + 3.0 , yolo_x, -yolo_y, 1.0;
                Eigen::Vector4d Pw_mid = Twb * Pc_mid;
                circleMsg circle_mid_position_world;
                circle_mid_position_world.pos.x = Pw_mid(0);
                circle_mid_position_world.pos.y = -Pw_mid(1);
                circle_mid_position_world.pos.z = Pw_mid(2);
                circle_msg_world_[closest_circle_num] = circle_mid_position_world;
            }
        }
    }
}

void circleTravelTask::droneSetGoalPosion(void) { //设置无人机目标点
    //每一帧开始先判断无人机是否到达目标障碍圈
    if(droneReachedLocation(circle_msg_world_[circle_num_].pos,drone_odom_,1.0)) {
        if(circle_num_ < 16) circle_num_++;
        if(circle_num_ == 8) circle_num_ = 12; //跳过剩余黄圈
    }
    //寻找是否拥有中间点
    auto mid_point = mid_point_map_.find(circle_num_);
    if(mid_point != mid_point_map_.end()) {
        //判断无人机是否到达中间点
        if(droneReachedLocation(mid_point->second,drone_odom_,2.0)) { //中间点到达条件可放宽一些
            mid_point_map_.erase(mid_point); //删除已到达的中间点
        }
        else {
            //以中间点为目标点
            drone_target_pose_.pose.position.x = mid_point->second.x;
            drone_target_pose_.pose.position.y = mid_point->second.y;
            drone_target_pose_.pose.position.z = mid_point->second.z;
        }
    }
    else { //当前圈没有中间点
        //更新目标点
        drone_target_pose_.pose.position.x = circle_msg_world_[circle_num_].pos.x;
        drone_target_pose_.pose.position.y = circle_msg_world_[circle_num_].pos.y;
        drone_target_pose_.pose.position.z = circle_msg_world_[circle_num_].pos.z;
    }
    
    //以无人机里程计的姿态角作为目标点的姿态角
    Eigen::Quaterniond quaternion(drone_odom_.pose.pose.orientation.w, drone_odom_.pose.pose.orientation.x, 
    drone_odom_.pose.pose.orientation.y, drone_odom_.pose.pose.orientation.z);
    drone_target_pose_.pose.orientation.w = quaternion.w();
    drone_target_pose_.pose.orientation.x = quaternion.x();
    drone_target_pose_.pose.orientation.y = quaternion.y();
    drone_target_pose_.pose.orientation.z = quaternion.z();

    if(abs(drone_target_pose_.pose.position.x) < 250 && abs(drone_target_pose_.pose.position.y) < 250 && abs(drone_target_pose_.pose.position.z) < 45) {
        drone_target_pose_.header.frame_id = "world";
        ego_goal_point_pub_.publish(drone_target_pose_);  
        // ROS_ERROR("ego,x:%f,y:%f,z:%f",drone_target_pose_.pose.position.x,drone_target_pose_.pose.position.y,drone_target_pose_.pose.position.z);
    }
}

//更新无人机状态 根据目标圈的长宽、倾斜度改变无人机的最大速度、控制参数
void circleTravelTask::droneStateUpdate(void){ 
    int ratio_flag = 0; //倾斜度标志 正对为0 略倾斜为1 很倾斜为2
    int threshold1 = 0, threshold2 = 0; //降速的长宽阈值
    if(circle_msg_world_[circle_num_].ratio < 0.6 || circle_msg_world_[circle_num_].ratio > 1.6) {
        ratio_flag = 2;
        threshold1 = 20;
        threshold2 = 40;
    }
    else if(circle_msg_world_[circle_num_].ratio > 0.8 || circle_msg_world_[circle_num_].ratio > 1.2) {
        ratio_flag = 1;
        threshold1 = 25;
        threshold2 = 50;
    }
    else {
        threshold1 = 30;
        threshold2 = 70;
    }
    float drone_max_vel,drone_max_acc;
    if(circle_msg_world_[circle_num_].width_max > threshold2) { //降到最低速模式
        drone_max_vel = MAX_VEL_SLOW;
        drone_max_acc = MAX_ACC_NORMAL;
    }
    else if(circle_msg_world_[circle_num_].width_max > threshold1) { //中速模式
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
    if(ego_init_flag_) { //ego初始化完成后，调用PD控制器
        airsim_interface_.airsimSetGoalPosition(ego_pos_cmd_.position.x,ego_pos_cmd_.position.y,
                                                ego_pos_cmd_.position.z,ego_pos_cmd_.yaw);
        
    }
}

int circleTravelTask::findClosestCircleNum(circleMsg circle_msg) {
    int closest_index = -1; // 初始化为无效值
    double minx_distance = 4.0; // 初始化为大于4米的值
    double miny_distance = 4.0;
    double minz_distance = 4.0;

    for (int i = circle_num_; i < circle_msg_ref_.size(); i++) {
        double xDistance = std::abs(circle_msg_ref_[i].pos.x - circle_msg.pos.x);
        double yDistance = std::abs(circle_msg_ref_[i].pos.y - circle_msg.pos.y);
        double zDistance = std::abs(circle_msg_ref_[i].pos.z - circle_msg.pos.z);

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
std::map<int,cv::Point3f> circleTravelTask::getMidPoint(void) {
    //清空中间点，避免重复叠加
    std::map<int,cv::Point3f> mid_point_map;
    int circle_num;
    cv::Point3f circle_mid_point;
    circle_num = 4;
    circle_mid_point.x = 105;
    circle_mid_point.y = -35;
    circle_mid_point.z = 5;
    mid_point_map.emplace(circle_num,circle_mid_point);
    circle_num = 12;
    circle_mid_point.x = -10;
    circle_mid_point.y = -50;
    circle_mid_point.z = 3;
    mid_point_map.emplace(circle_num,circle_mid_point);
    if(!circle_msg_ref_.empty()) { //以下中间点需要用到圈的参考位姿
        circle_num = 13;
        circle_mid_point.x = circle_msg_ref_[circle_num].pos.x;
        circle_mid_point.y = circle_msg_ref_[circle_num].pos.y;
        circle_mid_point.z = circle_msg_ref_[circle_num].pos.z;
        mid_point_map.emplace(circle_num,circle_mid_point);
        ROS_ERROR("push mid point circle%d, x:%f,y:%f,z:%f",circle_num,circle_mid_point.x,circle_mid_point.y,circle_mid_point.z);
    }
    return mid_point_map;
}

void circleTravelTask::visualOdometryCallBack(const nav_msgs::Odometry& drone_vins_odom){
    visual_odom_ = drone_vins_odom;
}

void circleTravelTask::egoPosCmdCallBack(const quadrotor_msgs::PositionCommand& ego_pos_cmd) {
    ego_pos_cmd_.position.x = ego_pos_cmd.position.x;
    ego_pos_cmd_.position.y = ego_pos_cmd.position.y;
    ego_pos_cmd_.position.z = ego_pos_cmd.position.z;
    ego_pos_cmd_.yaw = ego_pos_cmd.yaw;
    ego_pos_cmd_.velocity.x = ego_pos_cmd.velocity.x;
    ego_pos_cmd_.velocity.y = ego_pos_cmd.velocity.y;
    ego_pos_cmd_.velocity.z = ego_pos_cmd.velocity.z;
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