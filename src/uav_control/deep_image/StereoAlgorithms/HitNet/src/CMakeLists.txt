cmake_minimum_required (VERSION 3.8)

project(HitNet)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O0 -g -rdynamic -g2 -ggdb")

SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/uav_control/deep_image/StereoAlgorithms/HitNet/lib)
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/uav_control/deep_image/StereoAlgorithms/HitNet/include)
include_directories(${CMAKE_SOURCE_DIR}/uav_control/deep_image/StereoAlgorithms/common)
include_directories(${OpenCV_INCLUDE_DIRS})
# cuda
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

file(GLOB lib_header "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB common_header "${CMAKE_SOURCE_DIR}/uav_control/deep_image/StereoAlgorithms/common/*.h")

file(GLOB lib_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB common_src "${CMAKE_SOURCE_DIR}/uav_control/deep_image/StereoAlgorithms/common/*.cpp")


include_directories(${TensorRT_INCLUDE})
link_directories(${TensorRT_LIB})

cuda_add_library(${PROJECT_NAME} SHARED ${lib_src} ${common_src} HitNet_preprocess.cu)

# if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#     target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
# else()
#     target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/lib64/libcudart.so")
# endif()
target_link_libraries(${PROJECT_NAME} cudart)
target_link_libraries(${PROJECT_NAME} cudnn)
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvinfer.so")
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvonnxparser.so")
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvinfer_plugin.so")
#target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvinfer_builder_resource.so.8.6.1")
# target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libcudnn.so")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
