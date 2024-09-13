#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "model.h"
#include "utils.h"
#include "postprocess.h"
#include "preprocess.h"
#include "cuda_utils.h"
#include "logging.h"

using namespace nvinfer1;

class yolov8_seg
{
private:
    float gd = 0.0f, gw = 0.0f;
    int max_channels = 0;

    std::unordered_map<int, std::string> labels_map;
    std::string labels_filename = "../circle.txt";

    // Prepare cpu and gpu buffers
    cudaStream_t stream;
    float *device_buffers[3];
    float *output_buffer_host = nullptr;
    float *output_seg_buffer_host = nullptr;
    float *decode_ptr_host=nullptr;
    float *decode_ptr_device=nullptr;

    // engine 
    Logger gLogger;
    IRuntime *runtime = nullptr;
    IBuilder *builder = nullptr;
    IBuilderConfig *config = nullptr;
    ICudaEngine *engine = nullptr; // ICudaEngine* engine
    IExecutionContext *context = nullptr;
    IHostMemory *serialized_engine = nullptr;

    int model_bboxes;
    const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
    const static int kOutputSegSize = 32 * (kInputH / 4) * (kInputW / 4); 
public:
    yolov8_seg();
    cv::Rect get_downscale_rect(float bbox[4], float scale);
    std::vector<cv::Mat> process_mask(const float* proto, int proto_size, std::vector<Detection>& dets);
    void serialize_engine(std::string &wts_name, std::string &engine_name, std::string &sub_type);
    void prepare_buffer(std::string cuda_post_process);
    cv::Mat infer(cv::Mat& img, std::string cuda_post_process, std::vector<cv::Rect>& rects);
    bool parse_args(std::string &sub_type);
    ~yolov8_seg();
};

