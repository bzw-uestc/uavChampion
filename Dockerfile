#以nvdia官方cuda+cudnn+ubuntu20.04为基础镜像
FROM nvcr.io/nvidia/cuda:11.4.0-cudnn8-devel-ubuntu20.04
#安装ROS-noetic
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
RUN sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list
RUN apt-get clean && apt-get update --fix-missing
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
#安装ROS基础包
RUN apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
 ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
 net-tools libtool
#添加镜像中所需的本地库文件
ADD lib /uavChampion/lib/

#编译glog库
ENV ROS_DISTRO noetic
WORKDIR /uavChampion/lib/glog
RUN ./autogen.sh && ./configure && make && make install
RUN apt update && apt-get install -y liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
#编译ceres-solver库
WORKDIR /uavChampion/lib/ceres-solver-2.0.0rc1
RUN mkdir build && cd build && cmake .. && make -j16 && make install && apt-get install ros-noetic-ddynamic-reconfigure
#编译GPU版本的opencv4.5.5
RUN apt update && apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
 python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
WORKDIR /uavChampion/lib/opencv-4.5.5
RUN mkdir build && cd build 
WORKDIR /uavChampion/lib/opencv-4.5.5/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
 -D INSTALL_PYTHON_EXAMPLES=ON \
 -D INSTALL_C_EXAMPLES=ON \
 -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.5.5/modules \
 -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
 -D PYTHON_EXECUTABLE=/usr/bin/python \
 -D WITH_TBB=ON \
 -D WITH_V4L=ON \
 -D WITH_QT=OFF \
 -D WITH_GTK=ON \
 -D WITH_VTK=ON \
 -D WITH_OPENGL=ON \
 -D WITH_OPENMP=ON\
 -D BUILD_EXAMPLES=ON \
 -D WITH_CUDA=ON \
 -D BUILD_TIFF=ON \
 -D ENABLE_PRECOMPILED_HEADERS=OFF\
 -D INSTALL_PYTHON_EXAMPLES=ON \
 -D OPENCV_GENERATE_PKGCONFIG=ON \
 -D OPENCV_ENABLE_NONFREE=ON \
 -D CUDA_nppicom_LIBRARY=stdc++ \
 -D CUDA_ARCH_BIN="8.6" ..
RUN make -j16 && make install

#添加镜像中所需的本地代码文件
ADD src /uavChampion/src/
ADD setup.sh /
RUN chmod +x /setup.sh

#搭建生成特定yolo模型的环境 不上传时可注释
# RUN apt install -y python3-pip
# WORKDIR /fast_drone_ws/lib
# RUN chmod +x python_env.sh
# RUN ./python_env.sh

WORKDIR /uavChampion/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh  && catkin_make && . devel/setup.sh

ENTRYPOINT [ "/setup.sh" ]