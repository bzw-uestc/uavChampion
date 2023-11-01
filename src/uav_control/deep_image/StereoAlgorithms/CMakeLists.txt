cmake_minimum_required (VERSION 3.8)
project (StereoAlgorithmDemo)


# tensorrt path
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_INCLUDE "/usr/include/aarch64-linux-gnu" CACHE INTERNAL "TensorRT Library include location")
    set(TensorRT_LIB "/usr/lib/aarch64-linux-gnu" CACHE INTERNAL "TensorRT Library lib location")
else()
    set(TensorRT_INCLUDE "/home/uestc/TensorRT-8.4.3.1/targets/x86_64-linux-gnu/include" CACHE INTERNAL "TensorRT Library include location")
    set(TensorRT_LIB "/home/uestc/TensorRT-8.4.3.1/targets/x86_64-linux-gnu/lib" CACHE INTERNAL "TensorRT Library lib location")
endif()

find_package(CUDA REQUIRED)
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/home/uestc/TensorRT-8.4.3.1/include/)
link_directories(/home/uestc/TensorRT-8.4.3.1/lib/)


#添加子目
# add_subdirectory("RAFTStereo")
add_subdirectory("HitNet")
# add_subdirectory("CREStereo")
# add_subdirectory("FastACVNet_plus")  #如果编译请使用TensorRT-8.6+
# add_subdirectory("Stereo_Calibration")

