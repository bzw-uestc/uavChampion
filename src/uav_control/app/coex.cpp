#include "coex.hpp"


Coex::Coex() {}

float* blob = new float[480*640 * 3 * 2];

float* Coex::blobFromImage(cv::Mat& img1, cv::Mat& img2) {

  float mean[3] =  {0.485, 0.456, 0.406};
  float std[3] = {0.229, 0.224, 0.225};

  // float mean[3] =  {0.5, 0.5, 0.5};
  // float std[3] = {0.5, 0.5, 0.5};

  std::cout<< img1.total() << std::endl;
  int channels = 3;
  int img_h = img1.rows;
  int img_w = img1.cols;
  for (size_t c = 0; c < channels; c++) {
    for (size_t h = 0; h < img_h; h++) {
      for (size_t w = 0; w < img_w; w++) {
        blob[c * img_w * img_h + h * img_w + w] = (((float)img1.at<cv::Vec3b>(h, w)[c] / 255.0)- mean[c])/std[c];
        blob[c * img_w * img_h + h * img_w + w + img1.total() * 3] = (((float)img2.at<cv::Vec3b>(h, w)[c] / 255.0) - mean[c]) / std[c];
      }
    }
  }
  return blob;
}


void Coex::loadModel(char* model_path) {
  std::cout << model_path << std::endl;
  ifstream ifile(model_path, ios::in | ios::binary);
  if (!ifile) {
    cout << "read serialized file failed\n";
    std::abort();
  }

  ifile.seekg(0, ios::end);
  const int mdsize = ifile.tellg();
  ifile.clear();
  ifile.seekg(0, ios::beg);
  vector<char> buf(mdsize);
  ifile.read(&buf[0], mdsize);
  ifile.close();
  cout << "model size: " << mdsize << endl;

  runtime = nvinfer1::createInferRuntime(gLogger);

  initLibNvInferPlugins(&gLogger, "");
  engine = runtime->deserializeCudaEngine((void*)&buf[0], mdsize, nullptr);

  auto in_dims1 = engine->getBindingDimensions(engine->getBindingIndex("input_0"));  //1 3 480 640
  iH = in_dims1.d[2];
  iW = in_dims1.d[3];

  std::cout << in_dims1.d[1]<<iH << iW << std::endl;

  in_size1 = 1;
  for (int j = 0; j < in_dims1.nbDims; j++) {
    in_size1 *= in_dims1.d[j];
  }
  
  // auto in_dims2 = engine->getBindingDimensions(engine->getBindingIndex("input_1"));  //1 3 480 640
  // iH = in_dims2.d[2];
  // iW = in_dims2.d[3];
  // in_size2 = 1;
  // for (int j = 0; j < in_dims2.nbDims; j++) {
  //   in_size2 *= in_dims2.d[j];
  // }
  auto out_dims = engine->getBindingDimensions(engine->getBindingIndex("output_0")); // 1 480 640
  out_size = 1;
  for (int j = 0; j < out_dims.nbDims; j++) {
    out_size *= out_dims.d[j];
  }

  context = engine->createExecutionContext();
  if (!context) {
    cout << "create execution context failed\n";
    std::abort();
  }

  cudaError_t state;
  state = cudaMalloc(&buffs[0], in_size1 * sizeof(float));
  if (state) {
    cout << "allocate memory failed\n";
    std::abort();
  }
  // state = cudaMalloc(&buffs[1], in_size2 * sizeof(int));
  // if (state) {
  //   cout << "allocate memory failed\n";
  //   std::abort();
  // }

  state = cudaMalloc(&buffs[1], out_size * sizeof(float));
  if (state) {
    cout << "allocate memory failed\n";
    std::abort();
  }

  state = cudaStreamCreate(&stream);
  if (state) {
    cout << "create stream failed\n";
    std::abort();
  }

}

void Coex::Infer(cv::Mat &img1, cv::Mat &img2) {
  
  std::cout<< 0 << std::endl;
  // cv::cvtColor(img1, img1, cv::COLOR_BGR2RGB);
  // cv::cvtColor(img2, img2, cv::COLOR_BGR2RGB);
  // cv::imshow("img1", img1);
  // cv::imshow("img2", img2);
  std::cout<< 11 << std::endl;
  // std::cout<< img1.type() << CV_16U << std::endl;

  // 前处理 Normalize
  float* bolb = Coex::blobFromImage(img1, img2);

  cudaError_t state =
      cudaMemcpyAsync(buffs[0], bolb, in_size1 * sizeof(float), cudaMemcpyHostToDevice, stream);
  if (state) {
    cout << "transmit to device failed\n";
    std::abort();
  }
  std::cout<< 22 << std::endl;
  // state =
  //     cudaMemcpyAsync(buffs[1], &img2, in_size2 * sizeof(float), cudaMemcpyHostToDevice, stream);
  // if (state) {
  //   cout << "transmit to device failed\n";
  //   std::abort();
  // }
  std::cout<< 33 << std::endl;
  context->enqueueV2(&buffs[0], stream, nullptr);
  std::cout<< 44 << std::endl;
  state =
      cudaMemcpyAsync(deep_img, buffs[1], out_size * sizeof(float), cudaMemcpyDeviceToHost, stream);
  // std::cout<< deep_img[0] << std::endl;
  if (state) {
    cout << "transmit to host failed \n";
    std::abort();
  }
  std::cout<< 55 << std::endl;

  cudaStreamSynchronize(stream);

}

Coex::~Coex() {
  cudaStreamSynchronize(stream);
  cudaFree(buffs[0]);
  cudaFree(buffs[1]);
  cudaFree(buffs[2]);
  cudaStreamDestroy(stream);
  context->destroy();
  engine->destroy();
  runtime->destroy();
}

cv::Mat Coex::deep(const cv::Mat &cimg1, const cv::Mat &cimg2) {
  if (!(cimg1.empty() || cimg2.empty())) {
    // char* model_path = "Coexv5n.trt";
    cv::Mat img1 = cimg1;
    cv::Mat img2 = cimg2;
    auto start = std::chrono::system_clock::now();
    Coex::Infer(img1, img2);

    // Assuming you know the image dimensions (width, height, channels)
    int width = 640;  // Replace with actual width
    int height = 480; // Replace with actual height
    int channels = 1; // Replace with actual number of channels

    // Create a cv::Mat from the float array
    cv::Mat imageMat(height, width, CV_32FC1, deep_img);
    // cv::imshow("imageMat", imageMat);
    // cv::waitKey(1);
    auto end = std::chrono::system_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // ROS_ERROR("%dms",time);
    return imageMat * 1000;
  }
  else {
    std::cerr << "--> arguments not right!" << std::endl;
    std::cerr << "--> Coex -model_path ./output.trt -image_path ./demo.jpg" << std::endl;
  }



}
