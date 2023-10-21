#ifndef _DEEP_H
#define _DEEP_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <string.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <omp.h>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/imgproc/types_c.h>
#include "../../../lib/onnxruntime-linux-x64-gpu-1.16.1/include/onnxruntime_cxx_api.h"
#include "../../../lib/onnxruntime-linux-x64-gpu-1.16.1/include/onnxruntime_c_api.h"

// 命名空间
using namespace std;
using namespace cv;
using namespace Ort;



class Deep
{
public:
	Deep(char* modelpath);
    vector<float> BGR2RGB(Mat img);
	float* detect(cv::Mat& img1, cv::Mat& img2);
	float* det(cv::Mat& img1, cv::Mat& img2);
private:

	int inpWidth;
	int inpHeight;
	int nout;
	const bool keep_ratio = true;




	Env env = Env(ORT_LOGGING_LEVEL_ERROR, "Deep_img"); // 初始化环境
	Session *ort_session = nullptr;    // 初始化Session指针选项
	// Ort::Session *ort_session;
	SessionOptions sessionOptions = SessionOptions();  //初始化Session对象
	// SessionOptions sessionOptions;
	vector<char*> input_names;  // 定义一个字符指针vector

	vector<char*> output_names; // 定义一个字符指针vector
	vector<vector<int64_t>> input_node_dims; // >=1 outputs  ，二维vector
	vector<vector<int64_t>> output_node_dims; // >=1 outputs ,int64_t C/C++标准
};

#endif