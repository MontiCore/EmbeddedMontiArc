FROM ubuntu
 
#RUN apt remove -y *openjdk* 
#potential add more here 
RUN apt-get clean
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        wget python gcc \
        build-essential cmake \
        maven openjdk-8-jdk unzip
        

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y tzdata

RUN apt-get install -y --no-install-recommends python3 python3-pip python-dev python-setuptools python python-pip\
        libopencv-dev libarmadillo-dev

RUN pip install tensorflow==1.13.1   
RUN pip3 install opencv-python h5py numpy matplotlib scipy Pillow

RUN apt remove -y *openjdk* 
RUN apt-get install -y --no-install-recommends \
        openjdk-8-jdk

# Install tensorflow cc takes absolutly forever
# additionally to make it install you need at best 16 gb ram 
# if not availiable increase your swap memory to match it 
# from standard ubuntu 1gb to at least 8gb :) 
# alternatively, clone the repository
#RUN git clone https://github.com/FloopCZ/tensorflow_cc.git

# build and install
#RUN ./tensorflow_cc/Dockerfiles/install-ubuntu.sh --shared
