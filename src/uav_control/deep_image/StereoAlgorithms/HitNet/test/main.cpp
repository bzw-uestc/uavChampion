#include"HitNetAlgorithm.h"
#include<iostream>
#include<fstream> 
#include<chrono>


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


int main()
{
    char* stereo_calibration_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/FastACVNet_plus/test/StereoCalibrationUAV.yml";
    char* strero_engine_path="/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/model_float32.onnx";
    
    cv::Mat imageL=cv::imread("/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/im0.jpg");
    cv::Mat imageR=cv::imread("/home/uestc/uavChampion/src/uav_control/deep_image/StereoAlgorithms/HitNet/test/im1.jpg");
    
    //init
    void * raft_stereo=Initialize(strero_engine_path,0,stereo_calibration_path);

    float*pointcloud=new float[imageL.cols*imageL.rows*6];
    cv::Mat disparity;
    for (size_t i = 0; i < 10;i++)
    {
        cv::Mat imageL1=imageL.clone();
        cv::Mat imageR1=imageR.clone();
        //auto start = std::chrono::system_clock::now();
        RunHitNet(raft_stereo,imageL1,imageR1,pointcloud,disparity);
        //auto end = std::chrono::system_clock::now();
		//std::cout<<"time:"<<(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count())<<"ms"<<std::endl;
    }
    cv::imwrite("disparity.jpg",disparity);

    //heat map
    cv::Mat Heap_map=heatmap(disparity);
    cv::imwrite("heatmap.jpg",Heap_map);

    std::fstream pointcloudtxt;
    pointcloudtxt.open("pointcloud.txt",std::ios::out);
    for (size_t i = 0; i < imageL.cols*imageL.rows*6; i+=6)
    {
        pointcloudtxt<<pointcloud[i]<<" "<<pointcloud[i+1]<<" "<<pointcloud[i+2]<<" "
        <<pointcloud[i+3]<<" "<<pointcloud[i+4]<<" "<<pointcloud[i+5]<<std::endl;
    }
    pointcloudtxt.close();
    Release(raft_stereo);
    delete []pointcloud;
    pointcloud=nullptr;
    return 0;
}