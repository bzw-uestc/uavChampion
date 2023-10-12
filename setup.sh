#!/bin/bash

# 查看宿主机的显卡驱动（只能看到镜像自身的cuda版本）
nvidia-smi
nvcc -V

# 针对宿主机环境生成新的适配yolo模型 非上传时可注释
# cd lib
# echo 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.profile
# echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.profile
# echo 'export CUDA_HOME=$CUDA_HOME:/usr/local/cuda' >> ~/.profile
# echo 'export LD_LIBRARY_PATH=$PWD/TensorRT-8.6.1.6/lib:$LD_LIBRARY_PATH' >> ~/.profile
# echo 'export LIBRARY_PATH=$PWD/TensorRT-8.6.1.6/lib:$LIBRARY_PATH' >> ~/.profile
# source ~/.profile
# python3 TensorRT-For-YOLO-Series/export.py -o TensorRT-For-YOLO-Series/best.onnx -e TensorRT-For-YOLO-Series/yolov5n.trt --end2end
# cp TensorRT-For-YOLO-Series/yolov5n.trt ../src/uav_control/detect_model/
# cd ..

source devel/setup.bash
roslaunch uav_control run_all.launch & sleep 2
roslaunch uav_control run_vins.launch &
sleep 0.1
wait
exit 