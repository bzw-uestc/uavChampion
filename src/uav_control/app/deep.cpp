#include "deep.hpp"
#include <assert.h>
#include <ros/ros.h>


Deep::Deep(char* modelpath)
{
	this->inpHeight = 480;
	this->inpWidth = 640;

    OrtSessionOptionsAppendExecutionProvider_CUDA(sessionOptions, 0);
    sessionOptions.SetIntraOpNumThreads(1);

	sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);  //设置图优化类型

    const char* model_path = modelpath;

    ort_session = new Session(env, model_path, sessionOptions);


	size_t numInputNodes = ort_session->GetInputCount();  //输入输出节点数量                       
	size_t numOutputNodes = ort_session->GetOutputCount();
	AllocatorWithDefaultOptions allocator;   // 配置输入输出节点内存
    
	{
		// input_names.push_back(ort_session->GetInputName(i, allocator));		// 内存
        input_names.push_back("image1");		// 内存
		Ort::TypeInfo input_type_info1 = ort_session->GetInputTypeInfo(0);   // 类型
		auto input_tensor_info1 = input_type_info1.GetTensorTypeAndShapeInfo();  // 
		auto input_dims1 = input_tensor_info1.GetShape();    // 输入shape
		input_node_dims.push_back(input_dims1);	// 保存

        input_names.push_back("image2");		// 内存
	}
	for (int i = 0; i < numOutputNodes; i++)
	{
		// output_names.push_back(ort_session->GetOutputName(i, allocator));
        output_names.push_back("output");
		Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
		// auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
		// auto output_dims = output_tensor_info.GetShape();
		// output_node_dims.push_back(output_dims);
	}
	this->inpHeight = input_node_dims[0][2];
	this->inpWidth = input_node_dims[0][3];

}

vector<float> Deep::BGR2RGB(Mat img)  //归一化
{
	//    img.convertTo(img, CV_32F);
    //cout<<"picture size"<<img.rows<<img.cols<<img.channels()<<endl;
    
    vector<float> input_image_;
	int row = img.rows;
	int col = img.cols;
	input_image_.resize(row * col * img.channels());  // vector大小

	for (int c = 0; c < 3; c++)  // bgr
	{
		for (int i = 0; i < row; i++)  // 行
		{
			for (int j = 0; j < col; j++)  // 列
			{
				float pix = img.ptr<uchar>(i)[j * 3 + 2 - c];  // Mat里的ptr函数访问任意一行像素的首地址,2-c:表示rgb
				input_image_[c * row * col + i * col + j] = pix / 1.0; //将每个像素块归一化后装进容器
			}
		}
	}

    return input_image_;
}

float* Deep::detect(Mat& frame1, Mat& frame2)
{
	int newh = 0, neww = 0, padh = 0, padw = 0;

	// 定义一个输入矩阵，int64_t是下面作为输入参数时的类型
	array<int64_t, 4> input_shape_1{ 1, 3, 480, 640 };  //1,3,480,640
	array<int64_t, 4> input_shape_2{ 1, 3, 480, 640 };  //1,3,480,640
    

    // bgr2rgb
    vector<float> img1 = BGR2RGB(frame1);
    vector<float> img2 = BGR2RGB(frame2);
    // vector<float> img1(1*3*480*640);
    // vector<float> img2(1*3*480*640);
    //创建输入tensor
    /*
    这一行代码的作用是创建一个指向CPU内存的分配器信息对象(AllocatorInfo)，用于在运行时分配和释放CPU内存。
    它调用了CreateCpu函数并传递两个参数：OrtDeviceAllocator和OrtMemTypeCPU。
    其中，OrtDeviceAllocator表示使用默认的设备分配器，OrtMemTypeCPU表示在CPU上分配内存。
    通过这个对象，我们可以在运行时为张量分配内存，并且可以保证这些内存在计算完成后被正确地释放，避免内存泄漏的问题。
    */
	auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    //使用Ort库创建一个输入张量，其中包含了需要进行目标检测的图像数据。
	Value input_tensor_1 = Value::CreateTensor<float>(allocator_info, img1.data(), img1.size(), input_shape_1.data(), input_shape_1.size());
	Value input_tensor_2 = Value::CreateTensor<float>(allocator_info, img2.data(), img2.size(), input_shape_2.data(), input_shape_2.size());
	// assert(input_tensor_1.IsTensor());
	// assert(input_tensor_2.IsTensor());

	vector<Value> input_tensor_;
	input_tensor_.push_back(std::move(input_tensor_1));
	input_tensor_.push_back(std::move(input_tensor_2));
	auto start = std::chrono::system_clock::now();
	// 开始推理
	vector<Value> ort_outputs = ort_session->Run(RunOptions{nullptr}, input_names.data(), input_tensor_.data(), input_tensor_.size(), output_names.data(), output_names.size());   // 开始推理
	auto end = std::chrono::system_clock::now();
	int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	ROS_ERROR("tuili -> %dms",time);
	float* deep_img = ort_outputs[0].GetTensorMutableData<float>(); // GetTensorMutableData

	return deep_img;
}


float* Deep::det(cv::Mat& img1, cv::Mat& img2)
{
    float* deep_img = detect(img1, img2);
	// // 假设 data 是一个指向一维 float 数组的指针
	// const int rows = img1.rows; // 定义行数
	// const int cols = img1.cols; // 定义列数

	// // 创建 Mat 对象，并把 data 的数据拷贝进去
	// cv::Mat mat(rows, cols, CV_32FC1);
	// std::memcpy(mat.data, deep_img, rows * cols * sizeof(float));
	
	// cv::namedWindow("color", CV_WINDOW_NORMAL);
    // cv::imshow("color", mat);
    // cv::waitKey(1);
	return deep_img;
}
