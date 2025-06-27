<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Caffe2/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Caffe2/badges/master/coverage.svg)

# CNNArch2Caffe2
This generator generates caffe2 networks from the cnnarch language.


## Caffe2 Installation Guide 

This is a guide about how to install Caffe2 on Ubuntu and Windows 10. 

1. [Ubuntu](#ubuntu)
2. [Windows 10](#windows-10)


### Ubuntu 

This guide is based on the installation guide from the Caffe2 website: https://caffe2.ai/docs/getting-started.html?platform=ubuntu&configuration=compile

- **For GPU Support :** Install CUDA 8.0 and cuDNN directly from NVIDIA website.
    Install CUDA 8.0 with the option "deb (local)". Install cuDNN with the option "Library for Linux" and download cuDNN 7.0.5 since version 5.1 does not work with CUDA 8.0.
    
    Do the following in the extracted folder cuDNN 7.0.5:
    
    ```
    sudo cp -P include/cudnn.h /usr/local/cuda-8.0/include 
    sudo cp -P lib64/libcudnn* /usr/local/cuda-8.0/lib64
    ```

- Install Dependencies: 
	
	```
	sudo apt-get update
	sudo apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libgoogle-glog-dev \
      libgtest-dev \
      libiomp-dev \
      libleveldb-dev \
      liblmdb-dev \
      libopencv-dev \
      libopenmpi-dev \
      libsnappy-dev \
      libprotobuf-dev \
      openmpi-bin \
      openmpi-doc \
      protobuf-compiler \
      python-dev \
      python-pip                          
	pip install --user \
      future \
      numpy \
      protobuf
	```
	
	```
	#for Ubuntu 14.04
	sudo apt-get install -y --no-install-recommends libgflags2
	#for Ubuntu 16.04
	sudo apt-get install -y --no-install-recommends libgflags-dev
	```

- Set PYTHONPATH and LD_LIBRARY_PATH before compiling Caffe2. In terminal (anywhere), do the following: 
	
    ```
    sudo gedit ~/.bashrc
    ```
   
    Add the following at the end of the file. Don't forget to replace YourRepositoryPath (see below in clone Pytorch), e.g. by /home/carlos/Documents/git
	
    ```
    #caffe2
    #echo $PYTHONPATH
    export PYTHONPATH=/usr/local
    export PYTHONPATH=$PYTHONPATH:YourRepositoryPath/pytorch/build
    export PYTHONPATH=$PYTHONPATH:/usr/bin/python
    #echo $LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/usr/local/lib
    ```

    Save the file and close it. Then, in terminal do the following: 
	
    ```
    source ~/.bashrc
    ```
	
    Now, in terminal execute
	
    ```
    echo $PYTHONPATH  
    echo $LD_LIBRARY_PATH
    ```

    And review that there should not be double ":" in the paths.

- Clone Pytorch which now contains Caffe2 in your desired YourRepositoryPath and compile it:
	
	```
	git clone --recursive https://github.com/pytorch/pytorch.git && cd pytorch
	
	git checkout v0.4.0
        
	git submodule update --init
      
	mkdir build && cd build

	cmake ..
	```

	Review the output from cmake configuration. CUDA value should be CUDA 8.0 and cuDNN value should be 7.0.5.
    
	```
	sudo make install
	```

	Review that progress should achieve 100% and there should be no error messages.
	
- Test Caffe2 installation:
	In terminal (anywhere), do the following:
	
	```
	cd ~ && python -c 'from caffe2.python import core'
	2>/dev/null && echo "Success" || echo "Failure"
	```

    Review the output. Output should be Success.


### Windows 10

This guide is based on the installation guide from the Caffe2 website: https://caffe2.ai/docs/getting-started.html?platform=windows&configuration=compile

NOTE: PENDING TO CHECK WHETHER CAFFE2 IS SUPPORTED ON WINDOWS 10 BECAUSE LMDB WAS NOT DETECTED WHILE THIS GUIDE IS FOLLOWED

- Add the directories C:\Python27 and C:\Python27\Scripts to the system variable _Path_.

- Install python dependencies. Run the command line as administrator and navigate to path C:\Python27\Scripts. Execute the following commands:

	```
	pip install future
	pip install hypothesis	(if waring appears, then add C:\phyton27\Scripts to _Path_)
	pip install numpy 
	pip install protobuf
	pip install six
	```
	
	Optional packages:
	
	```
	pip install flask
	pip install glog
    pip install graphviz
	pip install jupyter
	pip install matplotlib
	pip install pydot python-nvd3
	pip install pyyaml
	pip install requests
	pip install scikit-image
	pip install scipy
	pip install setuptools
	pip install tornado
	```

    If there is an error installing optional python packages with “pip install …”, e.g., for graphviz or matplotlib, then execute the following in the same root:
	```
	pip install - - upgrade setuptools.
	```

- Install CMake (Add CMake bin to _Path_)

- Install Git from https://git-scm.com/downloads

- Clone Pytorch which now contains Caffe2 in your desired YourRepositoryPath:
	```
	git clone --recursive https://github.com/pytorch/pytorch.git 
	```
	
- Add the system variable CAFFE2_ROOT with the directory YourRepositoryPath\pytorch. Then, add the directory %CAFFE2_ROOT%\bin to _Path_.

    Afterwards, modify the file \scripts\build_windows.bat to set BUILD_PYTHON to ON instead of OFF.

    Open a Developer Command Prompt for VS 2017 (from Visual Studio 2017) and navigate to \scripts and execute build_windows.bat. Here the compilation will take place.
	
	```
	build_windows.bat
	```
	
- **For GPU Support :** Before executing build_windows.bat, set the following:

	```
	Set CMAKE_GENERATOR="Visual Studio 14 2015 Win64"
	Set USE_CUDA=ON
	Set TORCH_CUDA_ARCH_LIST=6.1 	//This applies for GeForce GTX 1050 architecture
	```
	
	Then execute build.windows.bat 

- Once Caffe2 is successfully compiled, add the system variable PYTHONPATH with the following directories in order to fix the possible error “ImportError: No module named Caffe2.python”:
	C:\Python27\Lib;
	C:\Phyton27\DLLs;
	C:\Phyton27\Lib\lib-tk;
	YourRepositoryPath\pytorch\build

## EMADL2CPP with CNNAcr2Caffe2

Once you managed to install Caffe2 you can clone the EMADL2CPP project https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP.git and install it
using `mvn clean install -s settings.xml`. This installs the generator into the .m2 directory of your maven nstallation (in Linux usually `/home/username/.m2`) under
`.m2\repository\de\monticore\lang\monticar\embedded-montiarc-emadl-generator\0.2.4\embedded-montiarc-emadl-generator-0.2.4-jar-with-dependencies.jar`.

You can run the generator with the following command `java -jar embedded-montiarc-emadl-generator-0.2.4-jar-with-dependencies.jar -m modelpath -r modelname -o output path -b CAFFE2`

## How to apply Data Cleaning
Missing, noisy and duplicate data can be classified as dirty data. Dirt data have an negative impact on the model performance. Therefore, one can apply data cleaning techniques like data removal to remove these kind of data entries. 
In order to apply data removal on our dataset, one can add a `cleaning` flag in the `Network.conf`:

```
cleaning: remove {
    missing:true
    duplicate:true
    noisy:true
}
```
The sub-parameters specifies if the corresponding dirty data type should be removed or not.

### Data Augmentation to counteract Data Imbalance

Data removal can cause data imbalance. To counter data imbalance, one can apply data up-sampling algorithms like data augmentation (e.g. image augmentation for images like the MNIST dataset). In order to apply image augmentation on our MNIST dataset, one can add a `data_augmentation` flag in the `Network.conf`:

```
data_imbalance: image_augmentation {
    rotation_angle:(-10,10,20) 
    shift:true
    scale_in:true
    scale_out:true
    check_bias:true
}
```
- `rotation_angle`: a list specifying the rotation degrees to be applied on an original image
- `shift`: speficies if a up, down, left and right shift should be applied
- `scale_in` and `scale_out`: specifies if the image should be scaled in and scaled out
- `check_bias`: specifies if the resulting up-sampled dataset should be checked for bias

