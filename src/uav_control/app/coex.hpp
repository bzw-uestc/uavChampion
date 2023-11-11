#ifndef _COEX_H
#define _COEX_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <array>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntimeCommon.h"
#include "NvOnnxParser.h"

using nvinfer1::Dims2;
using nvinfer1::Dims3;
using nvinfer1::IBuilder;
using nvinfer1::IBuilderConfig;
using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;
using nvinfer1::IHostMemory;
using nvinfer1::ILogger;
using nvinfer1::INetworkDefinition;
using Severity = nvinfer1::ILogger::Severity;

using cv::Mat;
using std::array;
using std::cout;
using std::endl;
using std::ifstream;
using std::ios;
using std::ofstream;
using std::string;
using std::vector;

class Logger_coex : public ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity != Severity::kINFO) {
      std::cout << msg << std::endl;
    }
  }
};

class Coex {
 public:
  Coex();
  void loadModel(char* model_path);
  ~Coex();
  void Infer(cv::Mat &img1, cv::Mat &img2);
  cv::Mat deep(const cv::Mat &cimg1, const cv::Mat &cimg2);
  float* blobFromImage(cv::Mat& img1, cv::Mat& img2);

 private:
  nvinfer1::ICudaEngine* engine = nullptr;
  nvinfer1::IRuntime* runtime = nullptr;
  nvinfer1::IExecutionContext* context = nullptr;
  cudaStream_t stream = nullptr;
  void* buffs[2];
  // float* deep_img {new float[480*640]};
  float deep_img[307200];
  int iH, iW, in_size1, in_size2, out_size;
  Logger_coex gLogger;
};


#endif // _COEX_H
