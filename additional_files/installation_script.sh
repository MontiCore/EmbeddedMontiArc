set -e

sudo apt-get update -y

sudo apt-get install -y lsb-release build-essential openjdk-8-jre gcc make cmake git python3 python3-dev python3-numpy swig libboost-all-dev curl

sudo apt-get install -y openjdk-8-jdk maven ninja-build ccache libopenblas-dev libblas-dev liblapack-dev libopencv-dev libarmadillo-dev

sudo apt-get install -y python3-tk python3-pip libboost-all-dev wget unzip

update-alternatives --config java

export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64

pip3 install --upgrade "cmake>=3.13.2"

pip3 install pyprind h5py matplotlib numpy mxnet==1.6.0

sudo apt-get install python2.7 python-dev python-numpy python-tk 

git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet
cd mxnet && git checkout tags/1.6.0 && git submodule update --recursive --init

mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=0 -GNinja .. && ninja -v && cd ..

sudo cp -r include/mxnet /usr/include/mxnet
sudo cp -r cpp-package/include/mxnet-cpp /usr/include
sudo cp -r 3rdparty/tvm/nnvm/include/nnvm /usr/include
sudo cp -r 3rdparty/dmlc-core/include/dmlc /usr/include
cp build/libmxnet.so ~/.local/lib/python3.8/site-packages/mxnet/

sudo cp -r build/libmxnet.so /usr/lib
cd ..
wget http://sourceforge.net/projects/arma/files/armadillo-9.900.5.tar.xz && tar -xf armadillo-9.900.5.tar.xz && cd armadillo-9.900.5 && cmake . && make 
sudo make install && cd ..

rm armadillo-9.900.5.tar.xz

mkdir -p .config/matplotlib
echo "backend : Agg" > .config/matplotlib/matplotlibrc
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add

sudo apt-get update -y

sudo apt-get install -y ros-noetic-desktop

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo rosdep init
rosdep update
