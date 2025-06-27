FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/integrationtests/mxnet:v0.0.3

RUN apt-get update
RUN apt-get install -y unzip g++
RUN apt-get install -y libopenblas-dev libhdf5-serial-dev

# Essential
RUN apt-get install -y openjdk-8-jdk openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl wget python-tk
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install h5py numpy pyprind matplotlib
RUN apt-get install -y python-matplotlib

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
RUN pip install mxnet

# Set Display for TKInter
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc
