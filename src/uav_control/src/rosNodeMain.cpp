#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include "../../cv_bridge/include/cv_bridge/cv_bridge.h" //opencv4.5.5所需的cv_bridge
#include <vector>
#include <queue>
#include <thread>
#include <utility>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudastereo.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include "cuda_runtime_api.h"
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntimeCommon.h"
#include "NvOnnxParser.h"
#include "../app/yoloV5.hpp"
#include "../app/coex.hpp"
#include "../app/circle_travel_task.hpp"
#include "../math/kalman_filter.hpp"
#include "../deep_image/StereoAlgorithms/HitNet/include/HitNetAlgorithm.h"
// #include "../app/deep.hpp"
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
cv::Mat getDepthFromHitNet(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline);

std::queue<sensor_msgs::ImageConstPtr> img_buf0;
std::queue<sensor_msgs::ImageConstPtr> img_buf1; //stereo_img
std::queue<geometry_msgs::Pose::ConstPtr> gps_buf;

int cnt_i = 0, cnt_j = 0;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_control_node");
  ros::NodeHandle nh("~");

  ros::Publisher stereo_pub = nh.advertise<sensor_msgs::Image>("/drone_0/depth", 1);
  ros::Subscriber sub_left_image,sub_right_image,sub_gps_msg;

  sub_left_image = nh.subscribe("/airsim_node/drone_1/front_left/Scene", 1, image0_callback);
  sub_right_image = nh.subscribe("/airsim_node/drone_1/front_right/Scene", 1, image1_callback);


  circleTravelTask circle_travel_task(nh);
  ros::Rate circle_travel_loop_rate(20);//设置循环频率，20Hz；也可以设为其他频率，如1为1Hz
  std::thread circle_travel_thread([&]() {
    while(ros::ok()) {
      circle_travel_task.circleTravelMain();
      circle_travel_loop_rate.sleep();
    }
  });

  // std::string package_path = ros::package::getPath("uav_control");
  // Coex coex_deep_;
  // std::string coex_model_path_str = package_path + "/detect_model/coex86.trt";
  // char* coex_model_path=const_cast<char*>(coex_model_path_str.c_str());
  // ROS_ERROR("coex_model_path:%s", coex_model_path);
  // coex_deep_.loadModel(coex_model_path); 


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
          //将时间同步后的图片传递到检测模块中
          std::vector<cv::Mat> img_raw_detect;
          img_raw_detect.push_back(ptr0->image);
          img_raw_detect.push_back(ptr1->image);
          circle_travel_task.img_detect_buf_.push(make_pair(color_msg0->header,img_raw_detect));

          //直接在该函数中进行深度图的计算
          cv::Mat gray_image_left,gray_image_right;
          cv::cvtColor(ptr0->image, gray_image_left, cv::COLOR_BGR2GRAY);
          cv::cvtColor(ptr1->image, gray_image_right, cv::COLOR_BGR2GRAY);
          cv::Mat disparity = getDepthFromStereo(gray_image_left,gray_image_right,320.0,95);   //use opencv to get deep
          cv::Mat img_depth = disparity2depth(disparity,320.0,95.0);
          sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", img_depth).toImageMsg();
          ///////////////////////////////////openCV SGBM///////////////////////////////////////////////////////

          // cv::Mat disparity = coex_deep_.deep(ptr0->image, ptr1->image); //use coex to get deep
          // disparity.convertTo(disparity, CV_16UC1);

          // cv::Mat disparity = getDepthFromHitNet(ptr0->image, ptr1->image, 320.0,95);             //use hitnet to get deep
          // cv::imshow("disparity", disparity);
          // cv::waitKey(1);

          
          // cv::Mat img_depth32f = disparity2depth_float(disparity,320.0,95.0);
          // sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", disparity).toImageMsg();

          rosDepthImage->header.stamp = color_msg0->header.stamp;  //同步深度图时间戳
          rosDepthImage->header.frame_id = "camera_depth";
          stereo_pub.publish(rosDepthImage);

          // //数据集的采集
          cnt_j++;
          if(cnt_j > 10) {
            cnt_j = 0;
            std::string name_left = "/home/uestc/bzw_ws/uavChampion/left/" + std::to_string(cnt_i) + ".jpg";
            // std::string name_right = "/home/uestc/bzw_ws/uavChampion/right/" + std::to_string(cnt_i) + ".jpg";
            cnt_i++;
            cv::imwrite(name_left, ptr0->image);
            // cv::imwrite(name_right, ptr1->image);
          }

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
  
  //heat map
  // cv::Mat Heap_map=heatmap(disparity);
  // cv::imshow("heatmap0.jpg",Heap_map);
  // cv::waitKey(1);

  return disparity;

  // cv::Mat depthMap16U = cv::Mat::zeros(img_left.size(), CV_16UC1); //深度图输出是CV_16UC1
  // depthMap16U = disparity2depth(disparity,fx,baseline);
  
  // ros::Time end = ros::Time::now();
  // // std::cout << end - start << std::endl;
  // return depthMap16U;
}



// /// @brief hitnet by zhuohui
// char* stereo_calibration_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/FastACVNet_plus/test/StereoCalibrationUAV.yml";
// char* strero_engine_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/model_float32.onnx";
// //init
// void * raft_stereo=Initialize(strero_engine_path,0,stereo_calibration_path);
// cv::Mat imageL=cv::imread("/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/im0.jpg");
// cv::Mat imageR=cv::imread("/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/im1.jpg");
// float*pointcloud=new float[imageL.cols*imageL.rows*6];
// // char* modelpath = "/home/uestc/mingwei/RAFT-Stereo/RAFT_Stereo.onnx";
// // Deep deep = Deep(modelpath);

// cv::Mat getDepthFromHitNet(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline) 
// {
//   cv::Mat disparity;
//   imageL = img_left;
//   imageR = img_right;

//   cv::Mat imageL1=imageL.clone();
//   cv::Mat imageR1=imageR.clone();
//   //auto start = std::chrono::system_clock::now();
//   RunHitNet(raft_stereo,imageL1,imageR1,pointcloud,disparity);


//   // float * deepimg = deep.det(imageL1, imageR1);
//   // cv::Mat imageMat(480, 640, CV_32FC3);
//   // memcpy(imageMat.data, deepimg, 640 * 480 * 1 * sizeof(float));
//   // cv::Mat Heap_map1=heatmap(imageMat);
//   // cv::imshow("Heap_map1.jpg",Heap_map1);
  
  
//   //auto end = std::chrono::system_clock::now();

//   //std::cout<<"time:"<<(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count())<<"ms"<<std::endl;
//   // cv::imshow("disparity.jpg",disparity);

//   //heat map
//   // cv::Mat Heap_map=heatmap(disparity);
//   // cv::imshow("heatmap1.jpg",Heap_map);
//   // cv::waitKey(1);


//   // std::fstream pointcloudtxt;
//   // pointcloudtxt.open("pointcloud.txt",std::ios::out);
//   // for (size_t i = 0; i < imageL.cols*imageL.rows*6; i+=6)
//   // {
//   //     pointcloudtxt<<pointcloud[i]<<" "<<pointcloud[i+1]<<" "<<pointcloud[i+2]<<" "
//   //     <<pointcloud[i+3]<<" "<<pointcloud[i+4]<<" "<<pointcloud[i+5]<<std::endl;
//   // }
//   // pointcloudtxt.close();
//   // Release(raft_stereo);
//   // delete []pointcloud;
//   // pointcloud=nullptr;
//   return disparity;
// }









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
          // depthData[id] = static_cast<unsigned short>((float)fx * baseline / ((float)dispData[id]));
          depthData[id] = static_cast<unsigned short>((float)dispData[id]);
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