#ifndef _ONNX2_H
#define _ONNX2_H

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
// #include "../../../lib/onnxruntime-linux-x64-gpu-1.16.1/include/tensorrt_provider_factory.h"
// #include "../../../lib/onnxruntime/include/onnxruntime/core/session/onnxruntime_c_api.h"
// #include "../../../lib/onnxruntime/include/onnxruntime/core/providers/cuda/cuda_provider_options.h"

// #include "../../../lib/onnxruntime/include/onnxruntime/core/session/onnxruntime_cxx_api.h"
// #include "../../../lib/onnxruntime/include/onnxruntime/core/session/onnxruntime_c_api.h"
// #include "../../../lib/onnxruntime/include/onnxruntime/core/providers/cuda/cuda_provider_options.h"

// 命名空间
using namespace std;
using namespace cv;
using namespace Ort;

// 自定义配置结构
struct Configuration
{
	public: 
	float confThreshold; // Confidence threshold置信度阈值
	float nmsThreshold;  // Non-maximum suppression threshold非最大抑制阈值
	float objThreshold;  //Object Confidence threshold对象置信度阈值
	char* modelpath;
};

// 定义BoxInfo结构类型
typedef struct BoxInfo
{
	float x1;
	float y1;
	float x2;
	float y2;
	float score;
	int label;
} BoxInfo;

// int endsWith(string s, string sub) {
// 	return s.rfind(sub) == (s.length() - sub.length()) ? 1 : 0;
// }

// const float anchors_640[3][6] = { {10.0,  13.0, 16.0,  30.0,  33.0,  23.0},
// 								 {30.0,  61.0, 62.0,  45.0,  59.0,  119.0},
// 								 {116.0, 90.0, 156.0, 198.0, 373.0, 326.0} };

// const float anchors_1280[4][6] = { {19, 27, 44, 40, 38, 94},{96, 68, 86, 152, 180, 137},{140, 301, 303, 264, 238, 542},
// 					   {436, 615, 739, 380, 925, 792} };

class YOLOv5
{
public:
	YOLOv5(Configuration config);
	vector<BoxInfo> detect(cv::Mat& img);
	vector<float> det(cv::Mat& img);
private:
	float confThreshold;
	float nmsThreshold;
	float objThreshold;
	int inpWidth;
	int inpHeight;
	int nout;
	int num_proposal;
	int num_classes;
	std::string classes[2] = {"red", "yellow"};
// string classes[80] = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus",
// 							"train", "truck", "boat", "traffic light", "fire hydrant",
// 							"stop sign", "parking meter", "bench", "bird", "cat", "dog",
// 							"horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
// 							"backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
// 							"skis", "snowboard", "sports ball", "kite", "baseball bat",
// 							"baseball glove", "skateboard", "surfboard", "tennis racket",
// 							"bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
// 							"banana", "apple", "sandwich", "orange", "broccoli", "carrot",
// 							"hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant",
// 							"bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
// 							"remote", "keyboard", "cell phone", "microwave", "oven", "toaster",
// 							"sink", "refrigerator", "book", "clock", "vase", "scissors",
// 							"teddy bear", "hair drier", "toothbrush"};


	const bool keep_ratio = true;
	vector<float> input_image_;		// 输入图片
	void normalize_(Mat img);		// 归一化函数
	void nms(vector<BoxInfo>& input_boxes);  
	Mat resize_image(Mat srcimg, int *newh, int *neww, int *top, int *left);

	Env env = Env(ORT_LOGGING_LEVEL_ERROR, "yolov5-6.1"); // 初始化环境
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