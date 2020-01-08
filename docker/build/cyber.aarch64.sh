
apt clean

# Add Bionic source
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted" > /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted" >> /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic universe" >> /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates universe" >> /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic multiverse" >> /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates multiverse" >> /etc/apt/sources.list
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list

#add Trusty universe into apt source for Poco foundation 9
echo "deb http://ports.ubuntu.com/ubuntu-ports/ trusty main" >> /etc/apt/sources.list
echo "deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe" >> /etc/apt/sources.list
echo "deb http://ports.ubuntu.com/ubuntu-ports/ xenial main" >> /etc/apt/sources.list
echo "deb http://ports.ubuntu.com/ubuntu-ports/ xenial universe" >> /etc/apt/sources.list

apt update -y && \
    apt install -y \
    build-essential \
    cmake \
    curl \
    gcc-5 \
    g++-5 \
    git \
    unzip \
    zip \
    vim \
    wget \
    bc \
    gdb \
    uuid-dev \
    python \
    python-dev \
    python3 \
    python3-dev \
    libasio-dev \
    libtinyxml2-6 \
    libtinyxml2-dev \
    libncurses5-dev \
    libavcodec57 \
    libavcodec-dev \
    libswscale4 \
    libswscale-dev \
    libcurl4-nss-dev \
    libpoco-dev \
    libconsole-bridge-dev \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    libpcap0.8 \
    libpcap0.8-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libopenni0 \
    libopenni-dev \
    libopenni2-0 \
    libopenni2-dev \
    openjdk-8-jdk \
    software-properties-common \
    libconsole-bridge-dev \
    liblz4-tool liblz4-dev liblz4-1 liblz4-1-dbg

# move eigen include
cp -r /usr/include/eigen3/Eigen /usr/local/include/
cp -r /usr/include/eigen3/unsupported /usr/local/include/

rm -f /usr/bin/gcc
ln -s /usr/bin/gcc-5 /etc/alternatives/gcc
ln -s /etc/alternatives/gcc /usr/bin/gcc
rm -f /usr/bin/g++
ln -s /usr/bin/g++-5 /etc/alternatives/g++
ln -s /etc/alternatives/g++ /usr/bin/g++

# Run installer [build|download]
cp -r docker/build/installers /tmp/installers
bash /tmp/installers/install_bazel.sh build
bash /tmp/installers/install_gflags_glog.sh
bash /tmp/installers/install_protobuf.sh
bash /tmp/installers/install_bazel_packages.sh
bash /tmp/installers/install_python_modules.sh
bash /tmp/installers/install_google_styleguide.sh
bash /tmp/installers/install_osqp.sh download
bash /tmp/installers/install_apriltag.sh
bash /tmp/installers/install_qt.sh
bash /tmp/installers/install_opencv4.1.sh
bash /tmp/installers/install_pcl.sh

apt update -y
apt install -y --allow-downgrades \
    libboost-system1.54.0 \
    libboost-thread1.54.0 \
    libboost-signals1.54.0 \
    libboost-filesystem1.54.0 \
    libboost-iostreams1.54.0 \
    libboost-chrono1.54.0 \
    libboost1.54-dev \
    libboost-dev=1.54.0.1ubuntu1 \
    libkml-dev \
    libgdal-dev \
    libvtk6-dev \
    libvtk6.3 \
    vtk6 \
    libpocofoundation9v5

rm -f /usr/lib/libPocoFoundation.so
ln -s /usr/lib/libPocoFoundation.so.9 /usr/lib/libPocoFoundation.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_iostreams.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_date_time.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_regex.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_serialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_serialization.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_signals.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_signals.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_system.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_system.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_wserialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_wserialization.so
ln -s /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so
ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.aarch64-linux-gnu.so /usr/lib/aarch64-linux-gnu/libvtkRenderingPythonTkWidgets.so

# intel realsense
bash /tmp/installers/install_realsense.sh
# install fast-rtps
bash /tmp/installers/install_fast-rtps.sh
rm -fr /tmp/*
