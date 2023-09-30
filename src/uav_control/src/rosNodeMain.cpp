#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
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

#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntimeCommon.h"
#include "NvOnnxParser.h"


void image0_callback(const sensor_msgs::ImageConstPtr &color_msg);
void image1_callback(const sensor_msgs::ImageConstPtr &color_msg);
void camera0Info_callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo);
void camera1Info_callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo);
void gpsCallback(const geometry_msgs::Pose::ConstPtr& gps_msg);
cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline);
void insertDepth32f(cv::Mat& depth);

std::queue<sensor_msgs::ImageConstPtr> img_buf0;
std::queue<sensor_msgs::ImageConstPtr> img_buf1; //stereo_img
sensor_msgs::CameraInfoConstPtr camera0_info;
sensor_msgs::CameraInfoConstPtr camera1_info;
std::queue<geometry_msgs::Pose::ConstPtr> gps_buf;
sensor_msgs::PointCloud2 rosPointCloud;

ros::Publisher camera0_info_pub,camera1_info_pub;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo2depth_node");
  ros::NodeHandle nh("~");

  ros::Publisher stereo_pub = nh.advertise<sensor_msgs::Image>("/drone_0/depth", 1);
  ros::Publisher gray_left_pub = nh.advertise<sensor_msgs::Image>("/narrow_stereo_textured/left/image_raw", 1); ///narrow_stereo_textured/left/image_raw
  ros::Publisher gray_right_pub = nh.advertise<sensor_msgs::Image>("/narrow_stereo_textured/right/image_raw", 1);
  camera0_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/narrow_stereo_textured/left/camera_info", 1);
  camera1_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/narrow_stereo_textured/right/camera_info", 1);

  ros::Publisher pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/drone_0_pcl_render_node/cloud", 1);
  ros::Publisher drone_true_odom_pub = nh.advertise<nav_msgs::Odometry>("/drone_true_odom", 1);  //发布出仿真器的真实位姿
  // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/odom", 10);
  
  ros::Publisher positionx_pub = nh.advertise<std_msgs::Float64>("/gps/postionx", 10);
  ros::Publisher positionkalx_pub = nh.advertise<std_msgs::Float64>("/gps/postionkalx", 10);
  ros::Publisher positiony_pub = nh.advertise<std_msgs::Float64>("/gps/postiony", 10);
  ros::Publisher positionkaly_pub = nh.advertise<std_msgs::Float64>("/gps/postionkaly", 10);
  ros::Publisher positionz_pub = nh.advertise<std_msgs::Float64>("/gps/postionz", 10);
  ros::Publisher positionkalz_pub = nh.advertise<std_msgs::Float64>("/gps/postionkalz", 10);
  
  tf2_ros::TransformBroadcaster* tf_broadcaster;
  ros::Subscriber sub_left_image,sub_right_image,sub_gps_msg;
  ros::Subscriber camera0_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/airsim_node/drone_1/front_left/Scene/camera_info", 1, camera0Info_callback);
  ros::Subscriber camera1_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/airsim_node/drone_1/front_right/Scene/camera_info", 1, camera1Info_callback);

  
  sub_left_image = nh.subscribe("/airsim_node/drone_1/front_left/Scene", 1, image0_callback);
  sub_right_image = nh.subscribe("/airsim_node/drone_1/front_right/Scene", 1, image1_callback);
  sub_gps_msg = nh.subscribe("/airsim_node/drone_1/pose", 10, gpsCallback);


  
  // 创建一个独立线程来运行数据处理线程
  std::vector<kalmanFilter> kal_gps_position;
  for(int i=0; i<3; ++i) { //创建xyz三轴gps位置卡尔曼滤波器
    double initial_position = 0.0; // 初始化位置
    double initial_Q = 9.0;       // 初始化过程噪声协方差，3的平方
    double initial_R = 1.0;       // 初始化测量噪声协方差
    kalmanFilter kf(initial_position, initial_Q, initial_R);
    kal_gps_position.push_back(kf);
  }

  uavControl drone0(nh);
  ros::Rate uavControl_loop_rate(100);//设置循环频率，20Hz；也可以设为其他频率，如1为1Hz
  std::thread uavControl_thread([&]() {
    while (ros::ok()) {
      drone0.uavControlTask();
      uavControl_loop_rate.sleep();
    }
  });

  char* model_path = "/home/uestc/bzw_ws/UAVChampion/fast-drone_ws/src/uav_control/detect_model/yolov5n.trt";
  Yolo yolo_detect(model_path);

  ros::Rate msgProcess_loop_rate(100);//设置循环频率，10Hz；也可以设为其他频率，如1为1Hz
  std::thread msgProcess_thread([&]() {
    while (ros::ok()) {
      if(!img_buf0.empty() && !img_buf1.empty()) {
        sensor_msgs::ImageConstPtr color_msg0 = nullptr;
        sensor_msgs::ImageConstPtr color_msg1 = nullptr;
        color_msg0 = img_buf0.front();
        color_msg1 = img_buf1.front();
        double t0 = color_msg0->header.stamp.toSec();
        double t1 = color_msg1->header.stamp.toSec();
        // if(t0 - t1 >= 0.05) {
        //   img_buf1.pop();
        //    ROS_ERROR("pop img1");
        //   break;
        // } 
        // else if(t1 - t0 >= 0.05) {
        //   img_buf0.pop();
        //   ROS_ERROR("pop img0");
        //   break;
        // }
        // else { //相机时间同步
          img_buf0.pop();
          img_buf1.pop();
          cv_bridge::CvImageConstPtr ptr0,ptr1; 

          ptr0 = cv_bridge::toCvCopy(color_msg0, sensor_msgs::image_encodings::BGR8);
          ptr1 = cv_bridge::toCvCopy(color_msg1, sensor_msgs::image_encodings::BGR8);// ros中的sensor_msg转成cv::Mat
          cv::Mat grayImageLeft,grayImageRight;
          cv::cvtColor(ptr0->image, grayImageLeft, cv::COLOR_BGR2GRAY);
          cv::cvtColor(ptr1->image, grayImageRight, cv::COLOR_BGR2GRAY);
          
          // cv::imshow("123",grayImageRight);
          // cv::imshow("456",grayImageLeft);
          /*重制图像话题为深度图 重制camerainfo 用于生成深度图*/
          sensor_msgs::ImagePtr gray_left_msg = cv_bridge::CvImage(color_msg0->header, "mono8", grayImageLeft).toImageMsg();
          gray_left_pub.publish(gray_left_msg);
          sensor_msgs::ImagePtr gray_right_msg = cv_bridge::CvImage(color_msg1->header, "mono8", grayImageRight).toImageMsg();
          gray_right_pub.publish(gray_right_msg);

          if(drone0.drone_pose_true_flag) {
            nav_msgs::Odometry drone_odom;
            drone_odom.header.frame_id = "world";
            drone_odom.header.stamp = color_msg0->header.stamp;
            drone_odom.pose.pose.position.x = drone0.drone_poses_true->position.x;
            drone_odom.pose.pose.position.y = drone0.drone_poses_true->position.y;
            drone_odom.pose.pose.position.z = -drone0.drone_poses_true->position.z;
            drone_odom.pose.pose.orientation.w = drone0.drone_poses_true->orientation.w;
            drone_odom.pose.pose.orientation.x = drone0.drone_poses_true->orientation.x;
            drone_odom.pose.pose.orientation.y = drone0.drone_poses_true->orientation.y;
            drone_odom.pose.pose.orientation.z = drone0.drone_poses_true->orientation.z;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(drone_odom.pose.pose.orientation, quat);
            double roll, pitch, yaw;//定义存储r\p\y的容器
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
            
            drone_true_odom_pub.publish(drone_odom);
          }

          // sensor_msgs::CameraInfo camera0_info_rebuild = *camera0_info;
          // camera0_info_rebuild.distortion_model = "plumb_bob";
          // camera0_info_rebuild.D = {0.0,0.0,0.0,0.0,0.0};
          // boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
          // camera0_info_rebuild.R = R;
          // camera0_info_rebuild.binning_x = 1;
          // camera0_info_rebuild.binning_y = 1;
          // camera0_info_rebuild.header = color_msg0->header;
          // camera0_info_rebuild.header.frame_id = "camera_init";
          // camera0_info_pub.publish(camera0_info_rebuild);

          // sensor_msgs::CameraInfo camera1_info_rebuild = *camera1_info;
          // boost::array<double, 12> P = {320.0, 0.0, 320.0, -95.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
          // camera1_info_rebuild.P = P;
          // camera1_info_rebuild.distortion_model = "plumb_bob";
          // camera1_info_rebuild.D = {0.0,0.0,0.0,0.0,0.0};
          // camera1_info_rebuild.R = R;
          // camera1_info_rebuild.binning_x = 1;
          // camera1_info_rebuild.binning_y = 1;
          // camera1_info_rebuild.header = color_msg1->header;
          // camera1_info_rebuild.header.frame_id = "camera_init";
          // camera1_info_pub.publish(camera1_info_rebuild);

          cv::Mat img_depth = getDepthFromStereo(grayImageLeft,grayImageRight,320.0,95);
          sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", img_depth).toImageMsg();
          rosDepthImage->header.stamp = color_msg0->header.stamp;  //同步深度图时间戳
          rosDepthImage->header.frame_id = "camera_depth";
          stereo_pub.publish(rosDepthImage);

          //left:ptr->image    right:ptr->right     cv::Mat
          std::vector<cv::Point>  detect_result;
          // Yolo yolo("yolov5n.trt");
          // cnt_j++;
          // if(cnt_j > 5) {
          //   cnt_j = 0;
          //   std::string name = "/home/uestc/uavChampion/dataset/" + std::to_string(cnt_i) + ".jpg";
          //   cnt_i++;
          //   cv::imwrite(name, ptr1->image);
          // }
          
          detect_result = yolo_detect.detect(ptr1->image);
          std::cout<< detect_result << std::endl;
        // }
      }
      // if(!gps_buf.empty()) {
      //   geometry_msgs::Pose::ConstPtr gps_msg = gps_buf.front();
      //   gps_buf.pop();
      //   double kalman_position_x = kal_gps_position[0].update(gps_msg->position.x);
      //   double kalman_position_y = kal_gps_position[0].update(gps_msg->position.y);
      //   double kalman_position_z = kal_gps_position[0].update(gps_msg->position.z);
      //   std_msgs::Float64 positionx,positiony,positionz;
      //   std_msgs::Float64 positionkalx,positionkaly,positionkalz;
      //   positionx.data = gps_msg->position.x;
      //   positiony.data = gps_msg->position.y;
      //   positionz.data = gps_msg->position.z;
      //   positionkalx.data = kalman_position_x;
      //   positionkaly.data = kalman_position_y;
      //   positionkalz.data = kalman_position_z;
      //   positionx_pub.publish(positionx);
      //   positiony_pub.publish(positiony);
      //   positionz_pub.publish(positionz);
      //   positionkalx_pub.publish(positionkalx);
      //   positionkaly_pub.publish(positionkaly);
      //   positionkalz_pub.publish(positionkalz);
      // }
      // 休眠以维持指定的频率
      msgProcess_loop_rate.sleep();
    }
  });
  
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

void camera0Info_callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
  camera0_info = cameraInfo;
  // ROS_INFO("receive image0");
}

void camera1Info_callback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
  camera1_info = cameraInfo;
  // ROS_INFO("rebuild camera1_info");
}

void gpsCallback(const geometry_msgs::Pose::ConstPtr& gps_msg) {
  gps_buf.emplace(gps_msg);
  // ROS_INFO("receive gps_msg");
}

cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline)
{
  ros::Time start = ros::Time::now();
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);   
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(img_left, img_right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
  insertDepth32f(disparity);
  cv::Mat disp8U = cv::Mat(disparity_sgbm.rows, disparity_sgbm.cols, CV_8UC1); // 创建一个用于显示的16位无符号整型cv::Mat对象
  // disparity_sgbm.convertTo(disp8U, CV_8UC1); // 将视差图转换为8位无符号整型
  disparity.convertTo(disp8U,CV_8UC1);

  cv::Mat depthMap16U = cv::Mat::zeros(disp8U.size(), CV_16UC1); //深度图输出是CV_16UC1
  int height = disp8U.rows;
  int width = disp8U.cols;
  uchar* dispData = (uchar*)disp8U.data; //视差图是CV_8UC1
  unsigned short* depthData = (unsigned short*)depthMap16U.data;
  for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
          int id = i*width + j;
          if (!dispData[id])  continue;  //防止0除
          depthData[id] = static_cast<unsigned short>((float)fx *baseline / ((float)dispData[id]));
      }
  }

  ros::Time end = ros::Time::now();
  std::cout << end - start << std::endl;
  return depthMap16U;
}

// cv::Mat getDepthFromStereo(const cv::Mat& img_left, const cv::Mat& img_right, const double& fx, const float& baseline) {
//   ros::Time start = ros::Time::now();
//   cv::cuda::GpuMat cudaLeftFrame, cudaRightFrame;
//   cudaLeftFrame.upload(img_left);
//   cudaRightFrame.upload(img_right); //复制图像到Cuda矩阵中
//   //参数：最小视差、 最大视差-最小视差、 P1P2控制平滑度
//   // cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
//   // cv::Ptr<cv::cuda::StereoSGM> sgbm = cv::cuda::createStereoSGM(0, 96, 648, 2592, 10, cv::cuda::StereoSGM::MODE_SGBM); //Cuda创建sgbm算法
//   cv::Ptr<cv::cuda::StereoSGM> sgbm = cv::cuda::createStereoSGM(0, 128, 8, 180, 3, cv::cuda::StereoSGM::MODE_HH); //Cuda创建sgbm算法
  
//   cv::cuda::GpuMat cudaDisparity_sgbm(img_left.size(), CV_16S); //视差图 CV_16S类型
//   cv::cuda::GpuMat cudaDisparity8U(img_left.size(), CV_8UC1); //视差图 CV_8UC1类型
//   cv::cuda::GpuMat cudaDisparity32F(img_left.size(), CV_32FC1); //视差图 CV_32FC1类型

//   sgbm->compute(cudaLeftFrame, cudaRightFrame, cudaDisparity_sgbm); //compute得到的视差图是CV_16S
  
//   // cudaDisparity_sgbm.convertTo(cudaDisparity32F,cudaDisparity32F.type()); //将SGM算法计算得到的视差图转换为CV_8UC1
//   // cv::Mat disp32F = cv::Mat(img_left.size(), CV_32FC1);
//   // cudaDisparity32F.download(disp32F);
//   // insertDepth32f(disp32F); //视差图空洞消除
//   // cv::Mat disp8U = cv::Mat(img_left.size(), CV_8UC1);
//   // disp32F.convertTo(disp8U,disp8U.type());

//   cudaDisparity_sgbm.convertTo(cudaDisparity8U,cudaDisparity8U.type()); //将SGM算法计算得到的视差图转换为CV_8UC1
//   cv::Mat disp8U = cv::Mat(img_left.size(), CV_8UC1); // 创建一个用于显示的16位无符号整型cv::Mat对象
//   cudaDisparity8U.download(disp8U);

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