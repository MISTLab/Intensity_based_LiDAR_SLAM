FROM ros:noetic-ros-base

LABEL Maintainer="Wenqiang Du <snowdwq@gmail.com>"
LABEL intensitySLAM.version="0.1"

RUN apt-get update 

RUN apt-get install -y  --fix-missing --no-install-recommends git ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-pcl-ros libpcl-dev wget unzip

RUN mkdir -p /dependencies_ws &&\
    cd /dependencies_ws &&\
    wget -O /dependencies_ws/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip &&\
    apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev unzip libsuitesparse-dev &&\
    cd /dependencies_ws/ && unzip ceres.zip -d /dependencies_ws/ &&\
    cd /dependencies_ws/ceres-solver-1.14.0 &&\
    mkdir ceres-bin && cd ceres-bin &&\
    cmake ..    &&\
    make install -j4

RUN apt update && apt install -y --force-yes apt-transport-https && \
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && \
    sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list' && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install cpio intel-mkl-64bit-2018.2-046
# RUN cd /dependencies_ws/ &&\
#     git clone https://github.com/borglab/gtsam.git
# RUN cd /dependencies_ws/gtsam &&\
#     mkdir build && cd build &&\
#     cmake .. &&\
#     make install -j4
RUN cd /dependencies_ws/ &&\
    wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.1.0.zip &&\
    unzip gtsam.zip -d /dependencies_ws/ &&\
    cd /dependencies_ws/gtsam-4.1.0 &&\
    mkdir build && cd build &&\
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. &&\
    make install -j4


RUN cd /dependencies_ws/ &&\
    git clone https://github.com/rmsalinas/DBow3.git &&\
    cd /dependencies_ws/DBow3 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make &&\
    make install -j4
RUN cp /usr/local/lib/libmetis-gtsam.so /usr/lib/

RUN mkdir -p /intensitySLAM/src &&\
    cd /intensitySLAM/src &&\
    git clone https://github.com/SnowCarter/Intensity_based_LiDAR_SLAM.git &&\
    cd /intensitySLAM &&\
    . /opt/ros/${ROS_DISTRO}/setup.sh  &&\
    catkin_make &&\
    echo "source /intensitySLAM/devel/setup.bash" >> ~/.bashrc

WORKDIR /intensitySLAM