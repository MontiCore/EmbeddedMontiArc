export DEBIAN_FRONTEND=noninteractive

# install general tools
apt-get update
apt-get -y install software-properties-common
add-apt-repository -y universe
apt-get install -y lsb-release build-essential openjdk-8-jre gcc make cmake git python3 python3-dev python3-numpy swig libboost-all-dev curl openjdk-8-jdk maven ninja-build ccache libopenblas-dev libblas-dev liblapack-dev libopencv-dev libarmadillo-dev python3-tk python3-pip libboost-all-dev wget unzip python-is-python3

# set correct java version
update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java

# update cmake
pip install --upgrade "cmake>=3.13.2"

# install mxnet pypi
pip install pyprind h5py matplotlib numpy mxnet==1.9.1 mxnet-cu112==1.9.1

# install mxnet cpp
git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet
cd mxnet && git checkout tags/1.9.1 && git submodule update --recursive --init
# https://github.com/apache/incubator-mxnet/blob/v1.9.1/config/linux_gpu.cmake
mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=1 -DMXNET_CUDA_ARCH="Pascal;Volta" -DUSE_CUDNN=1 -DUSE_NCCL=1 -DUSE_OPENCV=0 -DBUILD_CPP_EXAMPLES=0 -DUSE_OPERATOR_TUNING=1 -DUSE_GPERFTOOLS=0 -DUSE_JEMALLOC=0 -DUSE_SSE=1 -DUSE_F16C=0 -GNinja .. && ninja && cd ..
cp -r include/mxnet /usr/include/mxnet
cp -r cpp-package/include/mxnet-cpp /usr/include
cp -r 3rdparty/tvm/nnvm/include/nnvm /usr/include
cp -r 3rdparty/dmlc-core/include/dmlc /usr/include
cp -r build/libmxnet.so /usr/lib
cd ..

# install armadillo
wget http://sourceforge.net/projects/arma/files/armadillo-9.900.5.tar.xz && tar -xf armadillo-9.900.5.tar.xz && cd armadillo-9.900.5 && cmake . && make
make install && cd ..
rm armadillo-9.900.5.tar.xz

mkdir -p .config/matplotlib
echo "backend : Agg" > .config/matplotlib/matplotlibrc

# install ros
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add
apt-get update -y
apt-get install -y ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# setup ros
rosdep init
rosdep update
