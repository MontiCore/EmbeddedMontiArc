while true; do
# (c) https://github.com/MontiCore/monticore  
    read -p "This script will download, compile and install mxnet (in path ~/incubator-mxnet) and other requirements for emadl. It will create the virtualenv 'mxnet' for the mxnet python binding (usable with command 'source ~/mxnet/bin/activate'). Continue? (y/N)" yn1
    case $yn1 in
        [Yy]* ) break;;
        [Nn]* ) return;;
        * ) return;;
    esac
done
while true; do
    read -p "Do you wish to install mxnet with gpu support? Cuda and CudNN are required and have to be installed in path '/usr/local/cuda' for the gpu version. (y/n)" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) break;;
        * ) echo "Please answer y or n.";;
    esac
done
sudo apt-get update
sudo apt-get install -y build-essential git
sudo apt-get install -y libopenblas-dev liblapack-dev
sudo apt-get install -y libopencv-dev
sudo apt-get install -y python-dev python-setuptools python-pip libgfortran3
sudo apt-get install -y python3-pip
sudo apt-get install -y git 

yes | sudo pip3 install virtualenv
virtualenv -p /usr/bin/python2.7 ~/mxnet
source ~/mxnet/bin/activate

cd ~
git clone --recursive https://github.com/apache/incubator-mxnet
cd incubator-mxnet
make clean
case $yn in
    [Yy]* ) make -j $(nproc) USE_OPENCV=1 USE_BLAS=openblas USE_CUDA=1 USE_CUDA_PATH=/usr/local/cuda USE_CUDNN=1 USE_CPP_PACKAGE=1;;
    [Nn]* ) make -j $(nproc) USE_OPENCV=1 USE_BLAS=openblas USE_CPP_PACKAGE=1;;
    * ) echo "Please answer y or n.";;
esac
cd python
yes | pip install -e .
yes | pip install h5py
yes | pip install opencv-python
deactivate
