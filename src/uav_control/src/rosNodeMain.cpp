#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "../../cv_bridge/include/cv_bridge/cv_bridge.h" //opencv4.5.5所需的cv_bridge
#include <vector>
#include <queue>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudastereo.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include "../app/uav_control_task.hpp"
#include "../app/circleDetect.hpp"
#include "../math/kalman_filter.hpp"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntimeCommon.h"
#include "NvOnnxParser.h"
// #include "../app/onnx2.hpp"
// #include "../deep_image/StereoAlgorithms/FastACVNet_plus/include/FastACVNet_plus_Algorithm.h"
// #include"FastACVNet_plus_Algorithm.h"

void image0_callback(const sensor_msgs::ImageConstPtr &color_msg);
void image1_callback(const sensor_msgs::ImageConstPtr &color_msg);
void gpsCallback(const geometry_msgs::Pose::ConstPtr& gps_msg);
cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline);
void insertDepth32f(cv::Mat& depth);
cv::Mat disparity2depth(cv::Mat& disparity, float fx, float baseline);
cv::Mat disparity2depth_float(cv::Mat& disparity, float fx, float baseline);
cv::Mat heatmap(cv::Mat&disparity);

std::queue<sensor_msgs::ImageConstPtr> img_buf0;
std::queue<sensor_msgs::ImageConstPtr> img_buf1; //stereo_img

std::queue<geometry_msgs::Pose::ConstPtr> gps_buf;
sensor_msgs::PointCloud2 rosPointCloud;

int cnt_i = 500, cnt_j = 0;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_control_node");
  ros::NodeHandle nh("~");

  ros::Publisher stereo_pub = nh.advertise<sensor_msgs::Image>("/drone_0/depth", 1);
  ros::Publisher detect_left_pub = nh.advertise<sensor_msgs::Image>("/detect_image_left", 1);
  ros::Publisher drone_true_odom_pub = nh.advertise<nav_msgs::Odometry>("/drone_true_odom", 1);  //发布出仿真器的真实位姿
  // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/odom", 10);
  
  ros::Subscriber sub_left_image,sub_right_image,sub_gps_msg;

  sub_left_image = nh.subscribe("/airsim_node/drone_1/front_left/Scene", 1, image0_callback);
  sub_right_image = nh.subscribe("/airsim_node/drone_1/front_right/Scene", 1, image1_callback);

  std::string package_path = ros::package::getPath("uav_control");
  // std::string model_path_str = package_path + "/detect_model/yolov5n4090_2.trt";
  std::string model_path_str = package_path + "/detect_model/yolov5n_1025_lzh.trt";
  char* model_path=const_cast<char*>(model_path_str.c_str());
  ROS_ERROR("model_path:%s", model_path);
  Yolo yolo_detect(model_path);

  // std::string model2_path_str = package_path + "/detect_model/best1.onnx";
  // char* model_path2=const_cast<char*>(model2_path_str.c_str());
  // ROS_ERROR("model_path:%s", model_path2);
  // Configuration yolo_nets = { 0.3, 0.5, 0.3, model_path2 }; //初始化属性
  // ROS_ERROR("123");
	// YOLOv5 yolo_model(yolo_nets);

  // char* stereo_calibration_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/FastACVNet_plus/test/StereoCalibrationUAV.yml";
  // char* strero_engine_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/FastACVNet_plus/test/fast_acvnet_plus_generalization_opset16_480x640.onnx";
  // void * fastacvnet=Initialize(strero_engine_path,0,stereo_calibration_path);


  uavControl drone0(nh);
  ros::Rate uavControl_loop_rate(20);//设置循环频率，20Hz；也可以设为其他频率，如1为1Hz
  std::thread uavControl_thread([&]() {
    while(ros::ok()) {
      drone0.uavControlTask();
      uavControl_loop_rate.sleep();
    }
  });

  ros::Rate msgProcess_rate(50);
  std::thread msgProcess_thread([&]() {
    while(ros::ok()) {
      if(!img_buf0.empty() && !img_buf1.empty()) { //!img_buf0.empty() && !img_buf1.empty()   color_msg_left != nullptr && color_msg_right != nullptr
        auto start = std::chrono::system_clock::now();
        sensor_msgs::ImageConstPtr color_msg0 = nullptr;
        sensor_msgs::ImageConstPtr color_msg1 = nullptr;
        color_msg0 = img_buf0.front();
        color_msg1 = img_buf1.front();
        double t0 = color_msg0->header.stamp.toSec();
        double t1 = color_msg1->header.stamp.toSec();
        // ROS_ERROR("t0:%f,t1:%f",t0,t1);
        if(t0 - t1 >= 0.005) {
          img_buf1.pop();
          ROS_ERROR("pop img1");
          ROS_ERROR("t0:%f,t1:%f",t0,t1);
        } 
        else if(t1 - t0 >= 0.005) {
          img_buf0.pop();
          ROS_ERROR("pop img0");
          ROS_ERROR("t0:%f,t1:%f",t0,t1);
        }
        else { //相机时间同步
          img_buf0.pop();
          img_buf1.pop();
          cv_bridge::CvImageConstPtr ptr0,ptr1; 
          ptr0 = cv_bridge::toCvCopy(color_msg0, sensor_msgs::image_encodings::BGR8);
          ptr1 = cv_bridge::toCvCopy(color_msg1, sensor_msgs::image_encodings::BGR8);// ros中的sensor_msg转成cv::Mat
          cv::Mat grayImageLeft,grayImageRight;
          cv::cvtColor(ptr0->image, grayImageLeft, cv::COLOR_BGR2GRAY);
          cv::cvtColor(ptr1->image, grayImageRight, cv::COLOR_BGR2GRAY);

          if(drone0.drone_pose_true_flag) {
            nav_msgs::Odometry drone_odom;
            drone_odom.header.frame_id = "world";
            drone_odom.header.stamp = color_msg0->header.stamp;
            drone_odom.pose.pose.position.x = drone0.drone_poses_true->pose.position.x;
            drone_odom.pose.pose.position.y = -drone0.drone_poses_true->pose.position.y;
            drone_odom.pose.pose.position.z = -drone0.drone_poses_true->pose.position.z;
            drone_odom.pose.pose.orientation.w = drone0.drone_poses_true->pose.orientation.w;
            drone_odom.pose.pose.orientation.x = drone0.drone_poses_true->pose.orientation.x;
            drone_odom.pose.pose.orientation.y = -drone0.drone_poses_true->pose.orientation.y;
            drone_odom.pose.pose.orientation.z = -drone0.drone_poses_true->pose.orientation.z;
          
            drone_true_odom_pub.publish(drone_odom);
          }

          ///////////////////////////////////openCV SGBM///////////////////////////////////////////////////////
          cv::Mat disparity = getDepthFromStereo(grayImageLeft,grayImageRight,320.0,95);
          cv::Mat img_depth = disparity2depth(disparity,320.0,95.0);
          // cv::Mat img_depth32f = disparity2depth_float(disparity,320.0,95.0);
          sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", img_depth).toImageMsg();
          rosDepthImage->header.stamp = color_msg0->header.stamp;  //同步深度图时间戳
          rosDepthImage->header.frame_id = "camera_depth";
          stereo_pub.publish(rosDepthImage);


          // cnt_j++;
          // if(cnt_j > 5) {
          //   cnt_j = 0;
          //   std::string name_left = "/home/uestc/bzw_ws/uavChampion/left/" + std::to_string(cnt_i) + ".jpg";
          //   std::string name_right = "/home/uestc/bzw_ws/uavChampion/right/" + std::to_string(cnt_i) + ".jpg";
          //   cnt_i++;
          //   cv::imwrite(name_left, ptr0->image);
          //   // cv::imwrite(name_right, ptr1->image);
          // }

          /////////////////////////////////tensorRT////////////////////////////////////////////////////////////
          cv::Mat detect_img[2];
          detect_img[0] = ptr0->image.clone();
          detect_img[1] = ptr1->image.clone();
          // drone0.circle_detect_msg.clear();
          // drone0.circle_detect_msg = yolo_detect.detect(detect_img); //目标检测模块返回vector<float> 左上角x坐标、左上角y坐标、宽度、高度、类别名
          if(drone0.circle_msg_flag && drone0.takeoff_flag) {
            drone0.circle_detect_msg_queue.emplace(yolo_detect.detect(detect_img));
          }
          sensor_msgs::ImagePtr detect_image_left = cv_bridge::CvImage(color_msg0->header, "bgr8", detect_img[0]).toImageMsg();
          detect_left_pub.publish(detect_image_left);

          // cv::imshow("left",detect_img[0]);
          // cv::waitKey(1);
          //////////////////////////////////oonx/////////////////////////////////////////////////////////////////
          // std::vector<float> detect_temp0,detect_temp1; 
          // cv::Mat image_left = ptr0->image ,image_right = ptr1->image;
          // detect_temp0 = yolo_model.det(image_left);
          // detect_temp1 = yolo_model.det(image_right);
          // drone0.circle_detect_msg.push_back(detect_temp0);
          // drone0.circle_detect_msg.push_back(detect_temp1);
          // sensor_msgs::ImagePtr detect_image_left = cv_bridge::CvImage(color_msg0->header, "bgr8", image_left).toImageMsg();
          // detect_left_pub.publish(detect_image_left);


          drone0.image_left  = ptr0->image;
          drone0.image_right = ptr1->image;
          // drone0.image_depth = img_depth32f;

          auto end = std::chrono::system_clock::now();
          int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
          // ROS_ERROR("task1:%dms",time);
        }
      }
      msgProcess_rate.sleep();
    }
  });

  // std::thread fastACVNet_thread([&]() {
  //   while (ros::ok()) {
  //     if(!img_buf0_ACVNet.empty() && !img_buf1_ACVNet.empty()) { //!img_buf0.empty() && !img_buf1.empty()   color_msg_left != nullptr && color_msg_right != nullptr
  //       auto start = std::chrono::system_clock::now();
  //       sensor_msgs::ImageConstPtr color_msg0 = nullptr;
  //       sensor_msgs::ImageConstPtr color_msg1 = nullptr;
  //       color_msg0 = img_buf0_ACVNet.front();
  //       color_msg1 = img_buf1_ACVNet.front();
  //       double t0 = color_msg0->header.stamp.toSec();
  //       double t1 = color_msg1->header.stamp.toSec();
  //       if(t0 - t1 >= 0.005) {
  //         img_buf1_ACVNet.pop();
  //         ROS_ERROR("pop img1");
  //         break;
  //       } 
  //       else if(t1 - t0 >= 0.005) {
  //         img_buf0_ACVNet.pop();
  //         ROS_ERROR("pop img0");
  //         break;
  //       }
  //       else { //相机时间同步
  //         img_buf0_ACVNet.pop();
  //         img_buf1_ACVNet.pop();
  //         cv_bridge::CvImageConstPtr ptr0,ptr1; 
  //         ptr0 = cv_bridge::toCvCopy(color_msg0, sensor_msgs::image_encodings::BGR8);
  //         ptr1 = cv_bridge::toCvCopy(color_msg1, sensor_msgs::image_encodings::BGR8);// ros中的sensor_msg转成cv::Mat

  //         //FastACVNet
  //         float*pointcloud=new float[ptr0->image.cols*ptr0->image.rows*6];
  //         cv::Mat imageL1=ptr0->image.clone();
  //         cv::Mat imageR1=ptr1->image.clone();          
  //         // cv::Mat imageL1 = cv::imread("/home/uestc/uavChampion/left/80.jpg" );
  //         // cv::Mat imageR1 = cv::imread("/home/uestc/uavChampion/right/80.jpg");
  //         cv::Mat disparity;

  //         // cv::imshow("left",imageL1);
  //         // cv::imshow("right",imageR1);

  //         RunFastACVNet_plus_RectifyImage(fastacvnet,imageL1,imageR1,pointcloud,disparity);
  //         cv::Mat Heap_map = heatmap(disparity);
  //         // cv::imshow("disparity",disparity);
  //         // cv::imwrite("/home/uestc/uavChampion/disparity123.jpg",disparity);
  //         // 读取图像文件
  //         // cv::Mat Heap_map = cv::imread("/home/uestc/mingwei/StereoAlgorithms/build/heatmap80.jpg", cv::IMREAD_COLOR);

  //         // cv::Scalar color(0, 0, 255); // 蓝色 (BGR颜色)
  //         // // 画一个点
  //         // cv::circle(Heap_map, cv::Point2f(225, 220), 2, color, -1); // 5表示点的半径，-1表示填充点
  //         // cv::circle(Heap_map, cv::Point2f(330, 220), 2, color, -1); // 5表示点的半径，-1表示填充点
  //         // float left = pointcloud[(220*640+225)*6+2];
  //         // float right = pointcloud[(220*640+330)*6+2];
  //         // ROS_ERROR("%f,%f",left,right);
  //         // cv::Mat gray;
  //         // cv::cvtColor(Heap_map, gray, cv::COLOR_BGR2GRAY);
  //         // cv::imwrite("/home/uestc/uavChampion/gray111.jpg", gray);

  //         cv::Mat img_depth = cv::Mat::zeros(disparity.size(), CV_16UC1); //深度图输出是CV_16UC1
  //         img_depth = disparity2depth(disparity,320.0,95.0);
  //         cv::Mat img_depth32f = disparity2depth_float(disparity,320.0,95.0);
          
  //         // cv::imshow("img_depth",img_depth);
  //         // cv::imshow("disparity",disparity);

  //         // sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", img_depth).toImageMsg();
  //         // rosDepthImage->header.stamp = color_msg0->header.stamp;  //同步深度图时间戳
  //         // rosDepthImage->header.frame_id = "camera_depth";
  //         // stereo_pub.publish(rosDepthImage);
  //         drone0.heat_map = Heap_map;
  //         drone0.pointcloud = pointcloud;
  //         auto end = std::chrono::system_clock::now();
  //         int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  //         ROS_ERROR("task2:%dms",time);
  //       }
  //     }
  //   }
  // });

  ros::spin();
  return 0;
}


void image0_callback(const sensor_msgs::ImageConstPtr& color_msg) {
  img_buf0.emplace(color_msg);
  // ROS_INFO("receive image0");
}

void image1_callback(const sensor_msgs::ImageConstPtr& color_msg) {
  img_buf1.emplace(color_msg);
  // ROS_INFO("receive image1");
}

void gpsCallback(const geometry_msgs::Pose::ConstPtr& gps_msg) {
  // gps_buf.emplace(gps_msg);buf.emplace(gps_msg);
  // ROS_INFO("receive gps_msg");
}

// cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline)
// {
//   ros::Time start = ros::Time::now();
//   cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);   
//   cv::Mat disparity_sgbm, disparity;
//   sgbm->compute(img_left, img_right, disparity_sgbm);
//   disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
//   insertDepth32f(disparity);
//   cv::Mat disp8U = cv::Mat(disparity_sgbm.rows, disparity_sgbm.cols, CV_8UC1); // 创建一个用于显示的16位无符号整型cv::Mat对象
//   // disparity_sgbm.convertTo(disp8U, CV_8UC1); // 将视差图转换为8位无符号整型
//   disparity.convertTo(disp8U,CV_8UC1);

//   cv::Mat depthMap16U = cv::Mat::zeros(disp8U.size(), CV_16UC1); //深度图输出是CV_16UC1
//   int height = disp8U.rows;
//   int width = disp8U.cols;
//   uchar* dispData = (uchar*)disp8U.data; //视差图是CV_8UC1
//   unsigned short* depthData = (unsigned short*)depthMap16U.data;
//   for (int i = 0; i < height; i++) {
//       for (int j = 0; j < width; j++) {
//           int id = i*width + j;
//           if (!dispData[id])  continue;  //防止0除
//           depthData[id] = static_cast<unsigned short>((float)fx *baseline / ((float)dispData[id]));
//       }
//   }

//   ros::Time end = ros::Time::now();
//   // std::cout << end - start << std::endl;
//   return depthMap16U;

//   // float fy = 320.0,cx = 320.0,cy = 240.0;
//   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
// 	// for (int v = 0; v < img_left.rows; v++)
// 	// {
// 	// 	for (int u = 0; u < img_left.cols; u++)
// 	// 	{
// 	// 		if (disparity.at<float>(v, u) <= 10 || disparity.at<float>(v, u) >= 96) continue;
// 	// 		pcl::PointXYZRGB point;
// 	// 		double x = (u - cx) / fx;
// 	// 		double y = (v - cy) / fy;
// 	// 		double depth = fx * baseline / (disparity.at<float>(v, u));
// 	// 		point.x = x * depth;
// 	// 		point.y = y * depth;
// 	// 		point.z = depth;
// 	// 		point.b = img_left.at<cv::Vec3b>(v, u)[0];
// 	// 		point.g = img_left.at<cv::Vec3b>(v, u)[1];
// 	// 		point.r = img_left.at<cv::Vec3b>(v, u)[2];
// 	// 		pointcloud->push_back(point);
// 	// 	}
// 	// }
//   // pcl::visualization::PCLVisualizer visualizer("showcloud");
//   // while (!visualizer.wasStopped()) {
//   //   visualizer.removeAllPointClouds();
// 	//   visualizer.addPointCloud(pointcloud);
// 	//   visualizer.spinOnce();
//   // }
//   // cv::imshow("disparity", disparity);
//   // cv::waitKey(1);
// }

cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline) {
  ros::Time start = ros::Time::now();
  cv::cuda::GpuMat cudaLeftFrame, cudaRightFrame;
  cudaLeftFrame.upload(img_left);
  cudaRightFrame.upload(img_right); //复制图像到Cuda矩阵中
  //参数：最小视差、 最大视差-最小视差、 P1P2控制平滑度
  // cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  // cv::Ptr<cv::cuda::StereoSGM> sgbm = cv::cuda::createStereoSGM(0, 96, 648, 2592, 10, cv::cuda::StereoSGM::MODE_SGBM); //Cuda创建sgbm算法
  cv::Ptr<cv::cuda::StereoSGM> sgbm = cv::cuda::createStereoSGM(0, 128, 8, 180, 3, cv::cuda::StereoSGM::MODE_HH); //Cuda创建sgbm算法

  cv::cuda::GpuMat cudaDisparity_sgbm(img_left.size(), CV_16S); //视差图 CV_16S类型
  cv::cuda::GpuMat cudaDisparity(img_left.size(), CV_32F); //视差图 CV_32F类型
  sgbm->compute(cudaLeftFrame, cudaRightFrame, cudaDisparity_sgbm);
  cudaDisparity_sgbm.convertTo(cudaDisparity,CV_32F,1 / 16.0f);

  cv::Mat disparity;
  cudaDisparity.download(disparity);
  // insertDepth32f(disparity);
  return disparity;

  // cv::Mat depthMap16U = cv::Mat::zeros(img_left.size(), CV_16UC1); //深度图输出是CV_16UC1
  // depthMap16U = disparity2depth(disparity,fx,baseline);
  
  // ros::Time end = ros::Time::now();
  // // std::cout << end - start << std::endl;
  // return depthMap16U;
}


// cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline)
// {
//   ros::Time start = ros::Time::now();
//   cv::Mat disp;
//   disp.create(img_left.rows,img_right.cols,CV_16S);
//   cv::Mat disp1 = cv::Mat(img_left.rows,img_right.cols,CV_8UC1);
//   cv::Size imgSize = img_left.size();
//   cv::Rect roi1,roi2;
//   cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

//   int nmDisparities = ((imgSize.width / 8) + 15) & -16;//视差搜索范围

//   bm->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);//预处理滤波器类型
//   bm->setPreFilterSize(9);//预处理滤波器窗口大小
//   bm->setPreFilterCap(31);//预处理滤波器截断值
//   bm->setBlockSize(9);//SAD窗口大小
//   bm->setMinDisparity(0);//最小视差
//   bm->setNumDisparities(nmDisparities);//视差搜索范围
//   bm->setTextureThreshold(10);//低纹理区域的判断阈值
//   bm->setUniquenessRatio(5);//视差唯一性百分比
//   bm->setSpeckleWindowSize(100);//检查视差连通区域变化度窗口大小
//   bm->setSpeckleRange(32);//视差变化阈值
//   bm->setROI1(roi1);
//   bm->setROI2(roi2);
//   bm->setDisp12MaxDiff(1);//左右视差图最大容许差异
//   bm->compute(img_left,img_right,disp);

//   disp.convertTo(disp1,CV_8U,255 / (nmDisparities*16.));
//   cv::Mat depthMap;
//   depthMap.create(disp.rows,disp.cols,CV_8UC1);

//   cv::Mat depth1 = cv::Mat(disp.rows,disp.cols,CV_16S);
//   for (int i = 0;i < disp.rows;i++)
//   {
//       for (int j = 0;j < disp.cols;j++)
//       {
//           if (!disp.ptr<ushort>(i)[j])//防止除0中断
//               continue;
//           depth1.ptr<ushort>(i)[j] =  fx * baseline / disp.ptr<ushort>(i)[j];
//       }
//   }
//   depth1.convertTo(depthMap,CV_8U,1./256);//转8位

//   ros::Time end = ros::Time::now();
//   // std::cout << end - start << std::endl;
//   return depthMap;
// }

void insertDepth32f(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = MAX(0, left);
                right = MIN(right, width - 1);
                top = MAX(0, top);
                bot = MIN(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

cv::Mat disparity2depth(cv::Mat& disparity, float fx, float baseline) {
  // cv::Mat disp8U = cv::Mat(disparity.rows, disparity.cols, CV_8UC1);
  // disparity.convertTo(disp8U,CV_8UC1);
  
  cv::Mat depthMap16U = cv::Mat::zeros(disparity.size(), CV_16UC1); //深度图输出是CV_16UC1
  int height = disparity.rows;
  int width = disparity.cols;
  float* dispData = (float*)disparity.data; //视差图是CV_8UC1
  unsigned short* depthData = (unsigned short*)depthMap16U.data;
  for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
          int id = i*width + j;
          if (!dispData[id])  continue;  //防止0除
          depthData[id] = static_cast<unsigned short>((float)fx * baseline / ((float)dispData[id]));
      }
  }
  return depthMap16U;
}

cv::Mat disparity2depth_float(cv::Mat& disparity, float fx, float baseline) {
  // cv::Mat disp8U = cv::Mat(disparity.rows, disparity.cols, CV_8UC1);
  // disparity.convertTo(disp8U,CV_8UC1);
  
  cv::Mat depthMap32F = cv::Mat::zeros(disparity.size(), CV_32FC1); //深度图输出是CV_16UC1
  int height = disparity.rows;
  int width = disparity.cols;
  float* dispData = (float*)disparity.data; //视差图是CV_8UC1
  float* depthData = (float*)depthMap32F.data;
  for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
          int id = i*width + j;
          if (!dispData[id])  continue;  //防止0除
          depthData[id] = static_cast<float>((float)fx * baseline / ((float)dispData[id]));
      }
  }
  return depthMap32F;
}

cv::Mat heatmap(cv::Mat&disparity)
{
    //max min
    cv::Mat image_re = disparity.reshape(1);
    double minValue, maxValue;   
    cv::Point  minIdx, maxIdx;     
    cv::minMaxLoc(image_re, &minValue, &maxValue, &minIdx, &maxIdx);
    
    cv::Mat mean_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, minValue);
    cv::Mat std_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, (maxValue-minValue)/255);

    cv::Mat norm_disparity_map = (disparity - mean_mat) / std_mat;
    cv::Mat heap_map,abs_map;
    cv::convertScaleAbs(norm_disparity_map,abs_map,1);
    cv::applyColorMap(abs_map,heap_map,cv::COLORMAP_JET);
    return heap_map;

}