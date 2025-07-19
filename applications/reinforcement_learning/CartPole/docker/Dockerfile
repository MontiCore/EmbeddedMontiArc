FROM ros:kinetic
RUN apt-get update
RUN apt-get install -y unzip g++
RUN apt-get install -y libopenblas-dev libhdf5-serial-dev
ENV PATH="/usr/lib/x86_64-linux-gnu/:${PATH}"
ENV ROS_HOME=/opt/ros/kinetic
CMD ["bash"]

# Essential
RUN apt-get install -y openjdk-8-jdk openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl wget python-tk
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install h5py numpy pyprind matplotlib

# Armadillo
ADD armadillo-9.600.6.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/armadillo
RUN apt-get install -y libopenblas-dev liblapack-dev
RUN cd /root/armadillo/armadillo-9.600.6 && \
    cmake . && \
    make    && \
    make install

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
RUN rm -rf /usr/include/numpy && \
    ln -s "$(python -c 'import numpy; print(numpy.get_include())')/numpy" /usr/include

# Set Display for TKInter
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc