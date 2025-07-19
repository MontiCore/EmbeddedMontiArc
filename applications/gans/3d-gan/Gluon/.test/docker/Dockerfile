FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/integrationtests/mxnet:v0.0.5

RUN apt-get update --allow-releaseinfo-change
RUN apt-get install -y unzip g++
RUN apt-get install -y libopenblas-dev libhdf5-serial-dev

# Essential
RUN apt-get install -y gcc make cmake git python3.7 python3.7-dev python3.7-numpy swig libboost-all-dev curl wget python3.7-tk python3-pip
RUN pip3 install h5py numpy pyprind matplotlib
RUN pip3 install matplotlib

RUN rm -rf /root/armadillo
ADD armadillo-9.600.6.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/armadillo
RUN apt-get install -y libopenblas-dev liblapack-dev
RUN cd /root/armadillo/armadillo-9.600.6 && \
    cmake . && \
    make    && \
    make install

# MXNET Dependencies and MKLDNN
RUN apt-get install -y libopenblas-dev
RUN apt-get install -y libblas-dev liblapack-dev
RUN apt-get install -y build-essential
RUN apt-get install -y libopenblas-dev
RUN apt-get install -y libopencv-dev
RUN git clone https://github.com/oneapi-src/oneDNN.git
RUN cd oneDNN && mkdir -p build && cd build && \ 
    cmake .. && \
    make -j
RUN cd oneDNN && cd build && make install
ENV MKLDNN_ROOT /usr/local
ENV LD_LIBRARY_PATH $MKLDNN_ROOT/lib:$LD_LIBRARY_PATH

# MXNET PIP
RUN pip3 install --upgrade pip
RUN pip3 install mxnet

# Change default python version. We need python3
RUN rm -f /usr/bin/python && ln -s /usr/bin/python3 /usr/bin/python
# Set Display for TKInter
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc

