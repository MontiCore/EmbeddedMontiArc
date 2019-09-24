FROM ubuntu
 
#RUN apt remove -y *openjdk* 
#potential add more here 
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        wget python gcc \
        build-essential cmake \
        maven openjdk-8-jdk unzip  

ARG DEBIAN_FRONTEND=noninteractive
#RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get install -y tzdata

RUN apt-get clean 
RUN apt-get update
RUN apt-get install -y --no-install-recommends libopencv-dev 
RUN apt-get install -y --no-install-recommends libarmadillo-dev

RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python get-pip.py
RUN apt-get install -y --no-install-recommends python-dev python-setuptools

#potential add more here
RUN pip install tensorflow==1.13.1   
#RUN pip install opencv

#RUN pip  install opencv-python h5py numpy matplotlib scipy Pillow
RUN pip3 install opencv-python h5py numpy matplotlib scipy Pillow

# Install tensorflow cc takes absolutly forever
# additionally to make it install you need at best 16 gb ram 
# if not availiable increase your swap memory to match it 
# from standard ubuntu 1gb to at least 8gb :) 
# alternatively, clone the repository
#RUN git clone https://github.com/FloopCZ/tensorflow_cc.git

# build and install
#RUN ./tensorflow_cc/Dockerfiles/install-ubuntu.sh --shared

#Fails:

# java version 8 
# data transform script not working !!!
# Uninstalling tensorflow-estimator-1.14.0:

