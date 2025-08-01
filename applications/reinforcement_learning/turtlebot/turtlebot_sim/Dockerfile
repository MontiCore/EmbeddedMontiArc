# Dockerfile for ros + gazebo11 + mxnet

FROM ros:melodic

RUN apt update
RUN apt install -y unzip g++
RUN apt install -y libopenblas-dev libhdf5-serial-dev
ENV PATH="/usr/lib/x86_64-linux-gnu/:${PATH}"
ENV ROS_HOME=/opt/ros/melodic
CMD ["bash"]

# Essential
RUN apt install -y openjdk-8-jdk openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl wget python-tk
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install h5py numpy pyprind matplotlib

# Armadillo
RUN git clone --branch "9.900.x" https://gitlab.com/conradsnicta/armadillo-code.git armadillo_source
RUN apt install -y libopenblas-dev liblapack-dev
RUN cd /armadillo_source && \
    cmake . && \
    make    && \
    make install

# MXNET
RUN apt install -y libopenblas-dev
RUN apt install -y libblas-dev liblapack-dev
RUN apt install -y build-essential
RUN apt install -y libopenblas-dev
RUN apt install -y libopencv-dev
RUN git clone --branch 1.5.0 --recursive https://github.com/apache/incubator-mxnet.git mxnet_source
RUN cp -rL mxnet_source/include /usr/

RUN pip install mxnet==1.5.0
RUN cp -r "$(dirname $(python -c 'import mxnet; print(mxnet.__file__)'))/libmxnet.so" /usr/lib

#RUN cd mxnet_source/cpp-package/scripts && python OpWrapperGenerator.py libmxnet.so
RUN cp -rL mxnet_source/cpp-package/include /usr/


# Numpy Library
RUN rm -rf /usr/include/numpy && \
    ln -s "$(python -c 'import numpy; print(numpy.get_include())')/numpy" /usr/include

# Set Display for TKInter
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc

RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:openjdk-r/ppa -y
RUN apt-add-repository ppa:swi-prolog/stable -y
RUN apt update
RUN apt install openjdk-11-jdk maven swi-prolog dos2unix -y

#### Install Gazebo11 ###

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s -f /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11=11.12.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 11345

# setup entrypoint
COPY ./gzserver_entrypoint.sh /

##### install libgazebo11

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgazebo11-dev=11.12.0-1* \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir /environment
COPY . /environment
RUN bash -c "cd /environment && dos2unix *.sh"
RUN bash -c "cd /environment && ./config.sh"

EXPOSE 8080