FROM ros:kinetic
RUN apt-get update
RUN apt-get install -y unzip g++
RUN apt-get install -y libopenblas-dev libhdf5-serial-dev
ENV PATH="/usr/lib/x86_64-linux-gnu/:${PATH}"
ADD https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/
ENV Armadillo_HOME=/root/armadillo-8.500.1-linux
ENV ROS_HOME=/opt/ros/kinetic
CMD ["bash"]

# Essential
RUN apt-get install -y openjdk-8-jdk openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl wget python-tk
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install h5py numpy pyprind matplotlib

# Install Open CV - Warning, this takes absolutely forever
RUN apt-get update && apt-get install -y libgdal-dev
RUN mkdir -p ~/opencv cd ~/opencv && \
    wget https://github.com/Itseez/opencv/archive/3.0.0.zip && \
    unzip 3.0.0.zip && \
    rm 3.0.0.zip && \
    mv opencv-3.0.0 OpenCV
RUN apt-get install -y qtbase5-dev
RUN apt-get install -y build-essential
ENV PATH="${PATH}:/usr/include/"
RUN cd OpenCV && \
    mkdir build && \
    cd build && \
    cmake \
    -DWITH_QT=ON \
    -DWITH_OPENGL=ON \
    -DFORCE_VTK=ON \
    -DWITH_TBB=ON \
    -DWITH_GDAL=ON \
    -DWITH_XINE=ON \
    -DBUILD_EXAMPLES=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF ..
RUN cd OpenCV/build && make -j4
RUN cd OpenCV/build && make install
RUN cd OpenCV/build && ldconfig

# MXNET
RUN apt-get install -y libopenblas-dev
RUN apt-get install -y libblas-dev liblapack-dev
RUN apt-get install -y build-essential
RUN apt-get install -y libopenblas-dev
RUN apt-get install -y libopencv-dev
RUN git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet_source && \
    cd mxnet_source && git checkout tags/1.5.0 && cd .. && \
    cp -r mxnet_source/include/mxnet /usr/include/mxnet
RUN pip install mxnet
RUN cp -r "$(dirname $(python -c 'import mxnet; print(mxnet.__file__)'))/libmxnet.so" /usr/lib

# OpenAI Gym
RUN git clone https://github.com/openai/gym.git openai_gym && \
    cd openai_gym && \
    pip install -e . && \
    pip install -e '.[box2d]' && \
    pip install -e '.[atari]' && \
    cd ..

# Numpy Library
RUN if [ ! -d "/usr/include/numpy" && -L "/usr/include/numpy" ]; then cp -r "$(python -c 'import numpy; print(numpy.get_include())')/numpy" /usr/include; fi

# Set Display for TKInter
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc