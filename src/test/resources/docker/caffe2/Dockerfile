FROM maven:3-jdk-8

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   git \
   libgoogle-glog-dev \
   libgtest-dev \
   libiomp-dev \
   libleveldb-dev \
   liblmdb-dev \
   libopencv-dev \
   libopenmpi-dev \
   libsnappy-dev \
   libprotobuf-dev \
   openmpi-bin \
   openmpi-doc \
   protobuf-compiler \
   python-dev \
   python-pip    
RUN apt-get install -y --no-install-recommends python-setuptools                      
RUN pip install \
   future \
   numpy \
   protobuf
RUN apt-get install -y --no-install-recommends libgflags-dev

ENV NAME World
ENV PYTHONPATH="/usr/local"
ENV PYTHONPATH="${PYTHONPATH}:/pytorch/build"
ENV PYTHONPATH="${PYTHONPATH}:/usr/bin/python"
ENV LD_LIBRARY_PATH="/usr/local/lib"


RUN git clone --recursive https://github.com/pytorch/pytorch.git && cd pytorch
RUN cd pytorch && git checkout v0.4.0
RUN cd pytorch && git submodule update --init
RUN cd pytorch && mkdir build && cd build
RUN cd pytorch && cd build && cmake -DUSE_MPI=OFF ..
RUN cd pytorch && cd build && make install

RUN python -c 'from caffe2.python import core' 2>/dev/null && echo "Success" || echo "Failure"

RUN pip install lmdb

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        liblapack-dev libblas-dev libboost-dev libarmadillo-dev && \
    rm -rf /var/lib/apt/lists/* 

RUN python -c 'import lmdb' && echo "LMDB success" || echo "LMDB failure"



RUN apt remove nvidia-*
RUN apt update
RUN apt-key adv --fetch-keys  http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
RUN bash -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
RUN apt update
RUN apt install -y cuda-toolkit-9.2


ENV CUDNN_TAR_FILE="cudnn-8.0-linux-x64-v6.0.tgz"
RUN wget http://developer.download.nvidia.com/compute/redist/cudnn/v6.0/${CUDNN_TAR_FILE}
RUN tar -xzvf ${CUDNN_TAR_FILE}
RUN cp -P cuda/include/cudnn.h /usr/local/cuda/include
RUN cp -P cuda/lib64/libcudnn* /usr/local/cuda/lib64/
RUN chmod a+r /usr/local/cuda/lib64/libcudnn*