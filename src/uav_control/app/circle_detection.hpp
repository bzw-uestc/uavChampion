#ifndef _CIRCLE_DETECTION_H
#define _CIRCLE_DETECTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <queue>
#include <utility>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include "yoloV5.hpp"
#include "../../cv_bridge/include/cv_bridge/cv_bridge.h" //opencv4.5.5所需的cv_bridge


typedef struct {
    cv::Point3f pos;
    double yaw,ratio,width_max;
    int type;
}circleMsg;

class circleDetection{
private:
    enum selectPointEnum{upperMidpoint, downMidpoint, leftMidpoint, rightMidpoint};
    //检测主功能函数 检测当前帧下相机坐标系中障碍圈的位置 返回图像坐标系下所有障碍圈的位置 
    //YOLO检测器
    Yolo yolo_detect_;
    std::vector<std::vector<float>> yolo_detect_result_;
    //由双目恢复障碍圈相机坐标系下的三维坐标
    std::vector<double> detectCirclePosion(selectPointEnum point,const std::vector<float> left_msg,const std::vector<float> right_msg);
    //将图像坐标系中左右目的点转换为世界坐标系
    cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight);
    std::vector<cv::Point2f> ORBPointsMathch(cv::Mat& image_left, cv::Mat& image_right, std::vector<float>& ROI);
public:
    circleDetection(ros::NodeHandle &nh);
    ~circleDetection(){}
    //识别主函数 检测最新一帧图像中障碍圈相机坐标系下的位置
    std::vector<circleMsg> circleDetectionNewFrame(const cv::Mat* img_detect,float threshold_min, float threshold_max);

};

#endif
