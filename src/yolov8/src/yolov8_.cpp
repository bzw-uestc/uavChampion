#include "yolov8_.h"


cv::Rect yolov8_seg::get_downscale_rect(float bbox[4], float scale) {

  float left = bbox[0];
  float top  = bbox[1];
  float right  = bbox[0] + bbox[2];
  float bottom = bbox[1] + bbox[3];

  left    = left < 0 ? 0 : left;
  top     = top < 0 ? 0: top;
  right   = right > 640 ? 640 : right;
  bottom  = bottom > 640 ? 640: bottom;

  left   /= scale;
  top    /= scale;
  right  /= scale;
  bottom /= scale;
  return cv::Rect(int(left), int(top), int(right - left), int(bottom - top));
}

std::vector<cv::Mat> yolov8_seg::process_mask(const float* proto, int proto_size, std::vector<Detection>& dets) {

  std::vector<cv::Mat> masks;
  for (size_t i = 0; i < dets.size(); i++) {

    cv::Mat mask_mat = cv::Mat::zeros(kInputH / 4, kInputW / 4, CV_32FC1);
    auto r = get_downscale_rect(dets[i].bbox, 4);

    for (int x = r.x; x < r.x + r.width; x++) {
      for (int y = r.y; y < r.y + r.height; y++) {
        float e = 0.0f;
        for (int j = 0; j < 32; j++) {
            e += dets[i].mask[j] * proto[j * proto_size / 32 + y * mask_mat.cols + x];
        }
        e = 1.0f / (1.0f + expf(-e));
        mask_mat.at<float>(y, x) = e;
      }
    }
    cv::resize(mask_mat, mask_mat, cv::Size(kInputW, kInputH));
    masks.push_back(mask_mat);
  }
  return masks;
}

void yolov8_seg::serialize_engine(std::string &wts_name, std::string &engine_name, std::string &sub_type)
{
  std::ifstream file(engine_name, std::ios::binary);

  if (!file.good())
  {
    parse_args(sub_type);
    std::cout << "build engine!" << std::endl;
    std::cout<< "gd: "<<gd<<" gw: "<<gw<<std::endl;
    builder = createInferBuilder(gLogger);
    config = builder->createBuilderConfig();

    engine = buildEngineYolov8Seg(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);

    assert(engine);

    serialized_engine = engine->serialize();
    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p)
    {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());
    std::cout << "Build engine successfully!" << std::endl;

    
    context = engine->createExecutionContext();
    assert(context);

    delete serialized_engine;
    delete config;
    delete builder;

  }else{
    std::cout << "find engine, deserialize engine!" << std::endl; 
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char *serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    runtime = createInferRuntime(gLogger);
    assert(runtime);
    engine = runtime->deserializeCudaEngine(serialized_engine, size);
    assert(engine);
    context = engine->createExecutionContext();
    assert(context);
    delete[] serialized_engine;
  }

  auto out_dims = engine->getBindingDimensions(1);
  model_bboxes = out_dims.d[0];

}

void yolov8_seg::prepare_buffer(std::string cuda_post_process) {
    assert(engine->getNbBindings() == 3);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    const int outputIndex_seg = engine->getBindingIndex("proto");

    assert(inputIndex == 0);
    assert(outputIndex == 1);
    assert(outputIndex_seg == 2);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void **)&device_buffers[0], kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **)&device_buffers[1], kBatchSize * kOutputSize * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **)&device_buffers[2], kBatchSize * kOutputSegSize * sizeof(float)));
    
    if (cuda_post_process == "c") {
        output_buffer_host = new float[kBatchSize * kOutputSize];
        output_seg_buffer_host = new float[kBatchSize * kOutputSegSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void **)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }

    CUDA_CHECK(cudaStreamCreate(&stream));
}

cv::Mat yolov8_seg::infer(cv::Mat& img, std::string cuda_post_process, std::vector<cv::Rect>& rects) {
    cuda_preprocess_(img, device_buffers[0], kInputW, kInputH, stream);
    // infer on the batch asynchronously, and DMA output back to host
    context->enqueue(kBatchSize, (void**)device_buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        // std::cout << "kOutputSize:" << kOutputSize <<std::endl;
        CUDA_CHECK(cudaMemcpyAsync(output_buffer_host, device_buffers[1], kBatchSize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,stream));
        // std::cout << "kOutputSegSize:" << kOutputSegSize <<std::endl;
        CUDA_CHECK(cudaMemcpyAsync(output_seg_buffer_host, device_buffers[2], kBatchSize * kOutputSegSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float *)device_buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);//cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost, stream));
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));

    std::vector<Detection> res;
    cv::Mat img_masks;
    

    if (cuda_post_process == "c") {
            // NMS
            nms(res, output_buffer_host, kConfThresh, kNmsThresh);
            auto masks = process_mask(output_seg_buffer_host, kOutputSegSize, res);
            draw_mask_bbox(img, res, masks, labels_map, img_masks, rects);
            // cv::imshow("results" , img);
            // cv::waitKey(1);

        } else if (cuda_post_process == "g") {
            // Process gpu decode and nms results
            // batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
            // todo seg in gpu
            std::cerr << "seg_postprocess is not support in gpu right now" << std::endl;
        }
    
    return img_masks;
}

bool yolov8_seg::parse_args(std::string &sub_type)
{
    if (sub_type == "n") {
      gd = 0.33;
      gw = 0.25;
      max_channels = 1024;
    } else if (sub_type == "s") {
      gd = 0.33;
      gw = 0.50;
      max_channels = 1024;
    } else if (sub_type == "m") {
      gd = 0.67;
      gw = 0.75;
      max_channels = 576;
    } else if (sub_type == "l") {
      gd = 1.0;
      gw = 1.0;
      max_channels = 512;
    } else if (sub_type == "x") {
      gd = 1.0;
      gw = 1.25;
      max_channels = 640;
    } else{
      return false;
    }

    return true;
}

yolov8_seg::yolov8_seg(){
    read_labels(labels_filename, labels_map);
    assert(kNumClass == labels_map.size());
}

yolov8_seg::~yolov8_seg()
{
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(device_buffers[2]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    delete[] output_seg_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;

}

