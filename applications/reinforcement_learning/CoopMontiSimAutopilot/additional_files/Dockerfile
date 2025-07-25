# syntax=docker/dockerfile:1
FROM ubuntu:20.04

RUN export DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get -y install software-properties-common apt-utils
RUN add-apt-repository -y universe
RUN apt-get install -y lsb-release build-essential openjdk-8-jre gcc make cmake git python3 python3-dev python3-numpy swig libboost-all-dev curl openjdk-8-jdk maven ninja-build ccache libopenblas-dev libblas-dev liblapack-dev libopencv-dev libarmadillo-dev python3-tk python3-pip libboost-all-dev wget unzip python-is-python3

RUN update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java

RUN pip install --upgrade "cmake>=3.13.2"
RUN pip install pyprind h5py matplotlib numpy==1.19.1 mxnet==1.9.1
RUN git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet

RUN cd mxnet && git checkout tags/1.9.1 && git submodule update --recursive --init && mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=0 -DUSE_OPENCV=0 -DBUILD_CPP_EXAMPLES=0 -DUSE_OPERATOR_TUNING=1 -DUSE_GPERFTOOLS=0 -DUSE_JEMALLOC=0 -DUSE_SSE=1 -DUSE_F16C=0 -GNinja .. && ninja && cd .. && cp -r include/mxnet /usr/include/mxnet && cp -r cpp-package/include/mxnet-cpp /usr/include && cp -r 3rdparty/tvm/nnvm/include/nnvm /usr/include && cp -r 3rdparty/dmlc-core/include/dmlc /usr/include && cp -r build/libmxnet.so /usr/lib
RUN wget http://sourceforge.net/projects/arma/files/armadillo-9.900.5.tar.xz && tar -xf armadillo-9.900.5.tar.xz && cd armadillo-9.900.5 && cmake . && make && make install

RUN mkdir -p .config/matplotlib
RUN echo "backend : Agg" > .config/matplotlib/matplotlibrc


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add
RUN apt-get update -y
RUN apt-get install -y ros-noetic-desktop
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN rosdep init
RUN rosdep update

RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
RUN echo ". /opt/ros/noetic/setup.sh" >> /root/.bashrc
RUN echo "export ROS_PACKAGE_PATH=/opt/ros/noetic/share" >> /root/.bashrc
RUN echo "export CC=/usr/bin/gcc" >> /root/.bashrc
RUN echo "export CXX=/usr/bin/g++" >> /root/.bashrc
RUN echo "export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64" >> /root/.bashrc
