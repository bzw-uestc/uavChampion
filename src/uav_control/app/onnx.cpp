#include "onnx.hpp"
#include <ros/package.h>

using namespace cv;

Mat resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left)
{
    int srch = srcimg.rows, srcw = srcimg.cols;
    int inpHeight = 640;
    int  inpWidth = 640;
    *newh = inpHeight;
    *neww = 640;
    bool keep_ratio = true;
    Mat dstimg;
    if (keep_ratio && srch != srcw) {
        float hw_scale = (float)srch / srcw;
        if (hw_scale > 1) {
            *newh = inpHeight;
            *neww = int(inpWidth / hw_scale);
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *left = int((inpWidth - *neww) * 0.5);
            copyMakeBorder(dstimg, dstimg, 0, 0, *left, inpWidth - *neww - *left, BORDER_CONSTANT, 114);
        }
        else {
            *newh = (int)inpHeight * hw_scale;
            *neww = inpWidth;
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *top = (int)(inpHeight - *newh) * 0.5;
            copyMakeBorder(dstimg, dstimg, *top, inpHeight - *newh - *top, 0, 0, BORDER_CONSTANT, 114);
        }
    }
    else {
        resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
    }
    return dstimg;
}

void onnx_det(cv::Mat srcimg)
{
    //std::string imgpath = "images/bus.jpg";
    // std::string imgpath = "images/real.jpg";
    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_ERROR);//设置OpenCV只输出错误日志
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolov5s-5.0");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
    
    // OrtCUDAProviderOptions cuda_options{
    //       0,
    //       OrtCudnnConvAlgoSearch::OrtCudnnConvAlgoSearchExhaustive,
    //       std::numeric_limits<size_t>::max(),
    //       0,
    //       true
    //     };

    // session_options.AppendExecutionProvider_CUDA(cuda_options);

    // OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0);
    
    const char* model_path = "/home/uestc/mingwei/fast-drone_ws/best.onnx";
    
    
    Ort::Session session(env, model_path, session_options);
    // print model input layer (node names, types, shape etc.)
    // Ort::AllocatorWithDefaultOptions allocator;
    
    // print number of model input nodes
    size_t num_input_nodes = session.GetInputCount();
    std::vector<const char*> input_node_names = { "images"};
    std::vector<const char*> output_node_names = { "output0"};

    size_t input_tensor_size = 3*640*640;
    std::vector<float> input_tensor_values(input_tensor_size);
    // cv::Mat srcimg = cv::imread(imgpath);
    int newh = 0, neww = 0, padh = 0, padw = 0;
    
    Mat dstimg = resize_image(srcimg, &newh, &neww, &padh, &padw);//Padded resize
    
    //resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
	for (int c = 0; c < 3; c++)
	{
		for (int i = 0; i < 640; i++)
		{
			for (int j = 0; j < 640; j++)
			{
				float pix = dstimg.ptr<uchar>(i)[j * 3 + 2 - c];
                input_tensor_values[c * 640 * 640 + i * 640 + size_t(j)] = pix / 255.0;
			}
		}
	}
    // create input tensor object from data values
    std::vector<int64_t> input_node_dims = { 1, 3, 640, 640 };
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_size, input_node_dims.data(), input_node_dims.size());
    
    std::vector<Ort::Value> ort_inputs;
    ort_inputs.push_back(std::move(input_tensor));
    // score model & input tensor, get back output tensor
    std::vector<Ort::Value> output_tensors = session.Run(Ort::RunOptions{ nullptr }, input_node_names.data(), ort_inputs.data(), input_node_names.size(), output_node_names.data(), output_node_names.size());
    
    // Get pointer to output tensor float values
    const float* rawOutput = output_tensors[0].GetTensorData<float>();
	//generate proposals
    std::vector<int64_t> outputShape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    size_t count = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    std::vector<float> output(rawOutput, rawOutput + count);
    
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> classIds;
    int numClasses = (int)outputShape[2] - 5;
    int elementsInBatch = (int)(outputShape[1] * outputShape[2]);

    float confThreshold = 0.5;
    for (auto it = output.begin(); it != output.begin() + elementsInBatch; it += outputShape[2])
    {
        float clsConf = *(it+4);//object scores
        if (clsConf > confThreshold)
        {
            int centerX = (int)(*it);
            int centerY = (int)(*(it + 1));
            int width = (int)(*(it + 2));
            int height = (int)(*(it + 3));
            int x1 = centerX - width / 2;
            int y1 = centerY - height / 2;
            boxes.emplace_back(cv::Rect(x1, y1, width, height));

            // first 5 element are x y w h and obj confidence
            int bestClassId = -1;
            float bestConf = 0.0;

            for (int i = 5; i < numClasses + 5; i++)
            {
                if ((*(it + i)) > bestConf)
                {
                    bestConf = it[i];
                    bestClassId = i - 5;
                }
            }

            //confs.emplace_back(bestConf * clsConf);
            confs.emplace_back(clsConf);
            classIds.emplace_back(bestClassId);
        }
    }
    
    float iouThreshold = 0.5;
    std::vector<int> indices;
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    cv::dnn::NMSBoxes(boxes, confs, confThreshold, iouThreshold, indices);
    // std::cout << "789" << std::endl;
    //随机数种子
    // RNG rng((unsigned)time(NULL));
    for (size_t i = 0; i < indices.size(); ++i)
	{
        int index = indices[i];
        std::cout << index << std::endl;
        // int colorR = rng.uniform(0, 255);
        // int colorG = rng.uniform(0, 255);
        // int colorB = rng.uniform(0, 255);

        //保留两位小数
        float scores = round(confs[index] * 100) / 100;
        std::ostringstream oss;
        oss << scores;

		rectangle(dstimg, Point(boxes[index].tl().x, boxes[index].tl().y), Point(boxes[index].br().x, boxes[index].br().y), Scalar(0x27, 0xC1, 0x36), 1.5);
        
        // putText(dstimg, class_names[classIds[index]] + " " + oss.str(), Point(boxes[index].tl().x, boxes[index].tl().y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0xFF, 0xFF, 0xFF), 2);
        
	}
    
    imshow("results2", dstimg);
    cv::waitKey(1);
} 
