# TORCS DP RL
TORCS RL Controller based on the direct perception principle. A second computer vision DNN computes the affordance vectors used as inputs for the controller.

Content:
- [Prerequisites](#prerequisites)
    - [General](#general)
    - [Caffe](#caffe)
    - [MXNET](#mxnet)
    - [Armadillo](#armadillo)
    - [ROS](#ros)
    - [TORCS](#torcs)
- [Data Preprocessing](#data-preprocessing)
- [Code Generation and Instalation](#code-generation-and-instalation)
- [Training](#training)
- [Run Agent](#run-agent)


## Prerequisites
There is a singularity container available in order to train the model on the HPC. To create the container, follow the singularity [quick start guide](https://sylabs.io/guides/3.7/user-guide/quick_start.html#quick-installation-steps). Then run
```shell
sudo singularity build --sandbox RL-TORCS-Container aux/RL-TORCS-Container.def
```
Note, that the container does not include Caffe, which is needed for the data pre-processing.

### General
```shell
apt-get update -y 
apt-get install -y lsb-release build-essential openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl
apt-get install -y openjdk-8-jdk maven ninja-build ccache libopenblas-dev libblas-dev liblapack-dev libopencv-dev libarmadillo-dev
apt-get install -y python-tk python3 python3-pip libboost-all-dev wget unzip
pip3 install --upgrade "cmake>=3.13.2"
wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
python get-pip.py
pip install --user h5py matplotlib numpy==1.16.5 mxnet==1.5.1.post0
```

### Caffe
Caffe is only needed to convert the training data from levelDB to hdf5 in the preprocessing step.
For Ubuntu >= 17 run 
```shell
sudo apt-get install caffe-cpu
```

### MXNET
Clone MXNET
```shell
pip install mxnet
git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet
cd mxnet && git checkout tags/1.5.0 && git submodule update --recursive --init
```
Build MXNET
```shell
mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=0 -GNinja .. && ninja -v
```
Copy compiled library and include files 
```shell
cd .. && cp -r include/mxnet ../usr/include/mxnet && cp -r cpp-package/include/mxnet-cpp ../usr/include && cp -r 3rdparty/tvm/nnvm/include/nnvm ../usr/include && cp -r 3rdparty/dmlc-core/include/dmlc ../usr/include

cd ../usr/lib && cp -r ../../mxnet/build/libmxnet.so . && cd ../..
```

### Armadillo
```bash
wget http://sourceforge.net/projects/arma/files/armadillo-9.900.5.tar.xz && tar -xf armadillo-9.900.5.tar && cd armadillo-9.900.5 && cmake . && make && make install && cd ..
    
mkdir -p .config/matplotlib
echo "backend : Agg" > .config/matplotlib/matplotlibrc
```

### ROS
```bash
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add

sudo apt-get update -y

sudo apt-get install -y ros-kinetic-desktop

sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

rosdep init
rosdep update
rm /opt/ros/kinetic/etc/catkin/profile.d/1.ros_package_path.sh
rm /opt/ros/kinetic/etc/catkin/profile.d/99.roslisp.sh
```

### TORCS 
```bash
sudo apt-get install -y xautomation
sudo apt-get install -y libglib2.0-dev  libgl1-mesa-dev libglu1-mesa-dev  freeglut3-dev  libplib-dev libopenal-dev libalut-dev libxi-dev libxmu-dev libxrender-dev  libxrandr-dev libpng12-dev

pip install --ignore-installed gym
git clone https://github.com/ugo-nama-kun/gym_torcs.git
cd gym_torcs/vtorcs-RL-color/
./configure
make
make install
make datainstall
```

## Data Preprocessing
The [training data](http://deepdriving.cs.princeton.edu/) is available in the levelDB format which is not compatible with MXNET. The [test set](http://deepdriving.cs.princeton.edu/) is corrupted, which means, that the training set has to be split. During preprocessing, all labels are normalized. For preprocessing run 
```bash
python aux/torcs_data_preperation.py
```
**Attention:** change the paths in *torcs_data_preparation.py* before running it

## Code Generation and Instalation
To generate the code run 
```bash
./scripts/install-agent.sh
./scripts/install-affordance.sh
```

## Training
To train the agent run. Note: necessitates pre-trained affordance network loaded with affordance-component.
```
./scripts/train-agent.sh
```

To train the affordance network run 
```
./scripts/train-affordance.sh
```