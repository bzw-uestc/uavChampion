#include "circle_detection.hpp"


circleDetection::circleDetection(ros::NodeHandle &nh){
    ////////////////////////////////加载YOLO模型函数////////////////////////////////////
    std::string package_path = ros::package::getPath("uav_control");
    std::string model_path_str = package_path + "/detect_model/yolov5n4090trt84.trt";
    // std::string model_path_str = package_path + "/detect_model/yolov5n_1025_lzh.trt";
    char* model_path=const_cast<char*>(model_path_str.c_str());
    ROS_ERROR("model_path:%s", model_path);
    yolo_detect_.loadModel(model_path); 
}

std::vector<circleMsg> circleDetection::circleDetectionNewFrame(const cv::Mat* img_detect,float threshold_min, float threshold_max) {
    std::vector<circleMsg> circle_msg_return;
    yolo_detect_result_.clear();
    yolo_detect_result_ = yolo_detect_.detect(img_detect); //先用yolo检测器检测当前帧
    // if(!yolo_detect_result_.empty() && yolo_detect_result_[0].size() == yolo_detect_result_[1].size()) {
    //     for(int i = 0; i < yolo_detect_result_[0].size()/5; i++) { //遍历左目的每一个识别框
    //         // ROS_ERROR("yolo:x:%f,y:%f,width:%f,height:%f,type:%f",yolo_detect_result_[0][0],yolo_detect_result_[0][1],
    //         // yolo_detect_result_[0][2],yolo_detect_result_[0][3],yolo_detect_result_[0][4]);
    //         ////////////////////////////先计算识别框的信息 判断是否接近障碍圈/////////////////////////////////
    //         double circle_square = yolo_detect_result_[0][i*5+2] * yolo_detect_result_[0][i*5+3]; //识别框面积
    //         int circle_type = yolo_detect_result_[0][i*5+4]; //识别类型
    //         double circle_width = std::max(yolo_detect_result_[0][i*5+2] , yolo_detect_result_[0][i*5+3]); //长宽的较大值
    //         double circle_ratio = yolo_detect_result_[0][i*5+2] / yolo_detect_result_[0][i*5+3]; //计算障碍圈倾斜度
    //         std::vector<float> detect_msg_left,detect_msg_right;
    //         int start = i * 5,end = start + 4;
    //         // 复制数据到 detect_msg
    //         for(int j = 0; j < 5; j++) {
    //             detect_msg_left.push_back(yolo_detect_result_[0][i*5+j]);
    //             detect_msg_right.push_back(yolo_detect_result_[1][i*5+j]);
    //         }
    //         cv::Point2f circle_center;
    //         circle_center.x = detect_msg_left[0] + 0.5*detect_msg_left[2]; //左下角x坐标+宽度的一半
    //         circle_center.y = detect_msg_left[1] + 0.5*detect_msg_left[3]; //左下角x坐标+宽度的一半
    //         // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
    //         // cv::circle(img_detect[0], circle_center, 2, color, -1); // 5表示点的半径，-1表示填充点
    //         if(circle_width > threshold_min && circle_width < threshold_max) { //
    //             std::vector<double> up_point = detectCirclePosion(upperMidpoint,detect_msg_left,detect_msg_right);
    //             std::vector<double> down_point = detectCirclePosion(downMidpoint,detect_msg_left,detect_msg_right);
    //             std::vector<double> left_point = detectCirclePosion(leftMidpoint,detect_msg_left,detect_msg_right);
    //             std::vector<double> right_point = detectCirclePosion(rightMidpoint,detect_msg_left,detect_msg_right); //获取检测信息
    //             std::vector<cv::Point2f> circle_image_points = 
    //             {cv::Point2f(up_point[0],up_point[1]), cv::Point2f(down_point[0],down_point[1]), 
    //             cv::Point2f(left_point[0],left_point[1]), cv::Point2f(right_point[0],right_point[1])}; //图像的像素点坐标 pnp用
                
    //             if (up_point[2] != 0 && down_point[2] != 0 && left_point[2] != 0 && right_point[2] != 0) {
    //                 // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
    //                 // // 画一个点
    //                 // cv::circle(img_detect[0], circle_image_points[0], 2, color, -1); // 5表示点的半径，-1表示填充点
    //                 // cv::circle(img_detect[0], circle_image_points[1], 2, color, -1); // 5表示点的半径，-1表示填充点
    //                 // cv::circle(img_detect[0], circle_image_points[2], 2, color, -1); // 5表示点的半径，-1表示填充点
    //                 // cv::circle(img_detect[0], circle_image_points[3], 2, color, -1); // 5表示点的半径，-1表示填充点
    //                 // // 显示图像
    //                 // cv::imshow("result_left", img_detect[0]);
    //                 // // cv::imshow("result_right", image_right);
    //                 // cv::waitKey(1);

    //                 cv::Point3f circle_position_camera; 
    //                 circle_position_camera.x = (up_point[2] + down_point[2] + left_point[2] + right_point[2]) / 4;
    //                 circle_position_camera.y = (up_point[3] + down_point[3] + left_point[3] + right_point[3]) / 4;
    //                 circle_position_camera.z = (up_point[4] + down_point[4] + left_point[4] + right_point[4]) / 4;
    //                 if(circle_position_camera.z > 4.0) { //太近的观测舍弃
    //                     circleMsg circle_detect_msg;
    //                     circle_detect_msg.pos.x = circle_position_camera.x;
    //                     circle_detect_msg.pos.y = circle_position_camera.y;
    //                     circle_detect_msg.pos.z = circle_position_camera.z;
    //                     circle_detect_msg.width_max = circle_width;
    //                     circle_detect_msg.ratio = circle_ratio;
    //                     circle_detect_msg.type = circle_type;
    //                     circle_detect_msg.center = circle_center;
    //                     circle_detect_msg.square = circle_square;
    //                     circle_msg_return.push_back(circle_detect_msg);
    //                     // ROS_ERROR("camera x:%f,y:%f,z:%f",circle_detect_msg.pos.x,circle_detect_msg.pos.y,circle_detect_msg.pos.z);
    //                 }
    //             }
    //         }
    //     }
    // }
    if(!yolo_detect_result_.empty() && yolo_detect_result_[0].size() == yolo_detect_result_[1].size()) {
        for(int i = 0; i < yolo_detect_result_[0].size()/5; i++) { //遍历左目的每一个识别框
            // ROS_ERROR("yolo:x:%f,y:%f,width:%f,height:%f,type:%f",yolo_detect_result_[0][0],yolo_detect_result_[0][1],
            // yolo_detect_result_[0][2],yolo_detect_result_[0][3],yolo_detect_result_[0][4]);
            ////////////////////////////先计算识别框的信息 判断是否接近障碍圈/////////////////////////////////
            double circle_square = yolo_detect_result_[0][i*5+2] * yolo_detect_result_[0][i*5+3]; //识别框面积
            int circle_type = yolo_detect_result_[0][i*5+4]; //识别类型
            double circle_width = std::max(yolo_detect_result_[0][i*5+2] , yolo_detect_result_[0][i*5+3]); //长宽的较大值
            double circle_ratio = yolo_detect_result_[0][i*5+2] / yolo_detect_result_[0][i*5+3]; //计算障碍圈倾斜度
            std::vector<float> detect_msg_left,detect_msg_right;
            int start = i * 5,end = start + 4;
            // 复制数据到 detect_msg
            for(int j = 0; j < 5; j++) {
                detect_msg_left.push_back(yolo_detect_result_[0][i*5+j]);
                detect_msg_right.push_back(yolo_detect_result_[1][i*5+j]);
            }
            cv::Point2f circle_center;
            circle_center.x = detect_msg_left[0] + 0.5*detect_msg_left[2]; //左下角x坐标+宽度的一半
            circle_center.y = detect_msg_left[1] + 0.5*detect_msg_left[3]; //左下角x坐标+宽度的一半
            // cv::Scalar color(255, 0, 0); // 蓝色 (BGR颜色)
            // cv::circle(img_detect[0], circle_center, 2, color, -1); // 5表示点的半径，-1表示填充点
            
            if(circle_width > threshold_min && circle_width < threshold_max) { 
                circleMsg circle_detect_msg;
                circle_detect_msg.pos_camera.x = 0;
                circle_detect_msg.pos_camera.y = 0;
                circle_detect_msg.pos_camera.z = 0;
                circle_detect_msg.width_max = circle_width;
                circle_detect_msg.ratio = circle_ratio;
                circle_detect_msg.type = circle_type;
                circle_detect_msg.camera_center = circle_center;
                circle_detect_msg.square = circle_square;
                circle_msg_return.push_back(circle_detect_msg);
            }
        }
    }
    return circle_msg_return;
}

std::vector<double> circleDetection::detectCirclePosion(selectPointEnum point,const std::vector<float> left_msg,const std::vector<float> right_msg)
{
    std::vector<double> circle_detect_result; //存储检测结果变量
    // ORBPointsMathch(image_left,image_right,circle_detect_msg[0]);  //该方法先不用 等我回来调试
    int x1_left, y1_left, x1_right, y1_right;
    if (point == upperMidpoint) {
        x1_left = left_msg[0] + 0.5*left_msg[2]; //左下角x坐标+宽度的一半
        y1_left = left_msg[1] + 3; //左下角y坐标
        x1_right = right_msg[0] + 0.5 *right_msg[2];
        y1_right = right_msg[1] + 3;
    }
    else if (point == downMidpoint) {
        x1_left = left_msg[0] + 0.5 * left_msg[2]; //左下角x坐标+宽度的一半
        y1_left = left_msg[1] + left_msg[3] - 3; //左下角y坐标
        x1_right = right_msg[0] + 0.5 * right_msg[2];
        y1_right = right_msg[1] + right_msg[3] - 3;
    }
    else if (point == leftMidpoint) {
        x1_left = left_msg[0] + 3; //左下角x坐标
        y1_left = left_msg[1] + 0.5 * left_msg[3]; //左下角y坐标 + 高度一半
        x1_right = right_msg[0] + 3;
        y1_right = right_msg[1] + 0.5 * right_msg[3] ;
    }
    else if (point == rightMidpoint) {
        x1_left = left_msg[0] + left_msg[2] - 3; //左下角x坐标+宽度
        y1_left = left_msg[1] + 0.5 * left_msg[3] ; 
        x1_right = right_msg[0] + right_msg[2] - 3;
        y1_right = right_msg[1] + 0.5 * right_msg[3];
    }
    cv::Point2f left(x1_left , y1_left);
    cv::Point2f right(x1_right, y1_right);
    cv::Point3f circlePostionCamera = uv2xyz(left, right); //相机坐标系下圈的位姿
    circle_detect_result.push_back(x1_left);
    circle_detect_result.push_back(y1_left);
    circle_detect_result.push_back(circlePostionCamera.x);
    circle_detect_result.push_back(circlePostionCamera.y);
    circle_detect_result.push_back(circlePostionCamera.z);
    
    if(circlePostionCamera.x <12.0 && circlePostionCamera.y <12.0 && circlePostionCamera.z <12.0) {
        // return worldPoint;
        return circle_detect_result;
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
cv::Point3f circleDetection::uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight)
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
// Parameter: 左右目图像、ROI区域vector 左下角x坐标、左下角y坐标、宽度、高度、类名 
// Returns:   std::vector<cv::Point2f> 多对匹配点对
//************************************
std::vector<cv::Point2f> circleDetection::ORBPointsMathch(cv::Mat& image_left, cv::Mat& image_right, std::vector<float>& ROI) {
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