FROM ubuntu:16.04

	## --- --- Default Stuff --- --- ##
RUN apt-get update -y
RUN apt-get install -y wget cmake tar xz-utils build-essential curl git gedit

	## --- Install Java
RUN apt-get install -y default-jre default-jdk software-properties-common

	## --- --- Install Python --- ---
RUN apt install -y software-properties-common
RUN add-apt-repository -y ppa:deadsnakes/ppa
RUN apt-get update -y
RUN apt-get install -y wget python2.7 python-dev python3.7 python3-pip
RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
RUN python get-pip.py

RUN pip install \
    h5py \
    matplotlib \
    numpy==1.16.5 \
    mxnet==1.5

RUN python3.7 -m pip install --upgrade pip
RUN python3.7 -m pip install \
    numpy \
	matplotlib \
	mxnet==1.7.0.post1


	## --- --- Install MXNET --- ---
RUN cd / && apt-get install -y build-essential git ninja-build ccache libopenblas-dev libopencv-dev cmake gfortran 
RUN apt remove -y --purge --auto-remove cmake
RUN wget https://cmake.org/files/v3.14/cmake-3.14.0.tar.gz
RUN tar -xf cmake-3.14.0.tar.gz && cd cmake-3.14.0 && ./bootstrap && make && make install

RUN cd / && git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet && \
 	cd mxnet && git checkout tags/1.7.0 && git submodule update --recursive --init && \
 	mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=0 -GNinja .. && ninja -v && \
 	cd .. && cp -r include/mxnet ../usr/include/mxnet && cp -r cpp-package/include/mxnet-cpp ../usr/include && cp -r 3rdparty/tvm/nnvm/include/nnvm ../usr/include && cp -r 3rdparty/dmlc-core/include/dmlc ../usr/include && \
	cd ../usr/lib && cp -r ../../mxnet/build/libmxnet.so .



	## --- --- Install Armadillo --- ---
RUN cd / && apt-get install -y wget cmake tar xz-utils gcc build-essential
RUN wget http://sourceforge.net/projects/arma/files/armadillo-9.900.5.tar.xz && tar -xf armadillo-9.900.5.tar.xz && cd armadillo-9.900.5 && \
	mkdir build && cd build && cmake .. && make && make install && \
	cp libarmadillo* /usr/lib && cd ../include && cp -r * /usr/include && cd /usr



	## --- --- Install ROS Kinetic --- ---
RUN cd / && apt-get install -y software-properties-common curl

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add
RUN apt-get update -y
RUN apt-get install -y ros-kinetic-desktop-full
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

RUN rosdep init
RUN rosdep update
RUN rm /opt/ros/kinetic/etc/catkin/profile.d/1.ros_package_path.sh
RUN rm /opt/ros/kinetic/etc/catkin/profile.d/99.roslisp.sh
	


	## --- --- Upgrade GCC --- ---
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get -y update
RUN apt-get -y install gcc-7 g++-7
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7



	## --- --- Install  TORCS --- ---
RUN apt-get -y update && \
 	cd / && apt-get install -y git xautomation libglib2.0-dev  libgl1-mesa-dev libglu1-mesa-dev  freeglut3-dev  libplib-dev  libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev  libxrandr-dev libpng12-dev build-essential && \
	cd / && apt-get install -y git wget unzip patch xautomation libglib2.0-dev  libgl1-mesa-dev libglu1-mesa-dev  freeglut3-dev  libplib-dev  libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev  libxrandr-dev libpng12-dev build-essential libvorbis-dev libpng-dev libtinyxml-dev mesa-common-dev 
	
RUN git clone https://github.com/ugo-nama-kun/gym_torcs && \
    cd gym_torcs/vtorcs-RL-color && \
	## patching src
	cd src/drivers/olethros && wget https://sourceforge.net/p/torcs/support-requests/11/attachment/torcs-1.3.7-isnan.patch && patch < torcs-1.3.7-isnan.patch && cd ../..

RUN cd /gym_torcs/vtorcs-RL-color && \
	./configure && \
	make && \
	make install && \
	make datainstall

	##cd /    
	##wget http://deepdriving.cs.princeton.edu/DeepDrivingCode_v2.zip
	##unzip DeepDrivingCode_v2.zip
	##cd DeepDriving/torcs-1.3.6
	##cp -rf ../modified_tracks/* /opt/torcs/share/games/torcs/tracks/road

## RUN	source /opt/ros/kinetic/setup.bash
RUN	export ROS_PACKAGE_PATH=/opt/ros/kinetic/share
