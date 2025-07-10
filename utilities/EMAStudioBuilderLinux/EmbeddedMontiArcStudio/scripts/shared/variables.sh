pushd `pwd` > /dev/null
# (c) https://github.com/MontiCore/monticore  
cd ..
cd ..
if !([[ -v USER_HOME ]])
then export USER_HOME=~
fi
export HOME=`pwd`
popd  > /dev/null
export SIMULATION_URL=http://localhost:8080/
export JAVA_HOME=${HOME}/jdk-linux
export TOMCAT_HOME=${HOME}/apache-tomcat-9.0.5
export MINGW_HOME=${HOME}/mingw64
export OCTAVE_HOME=${HOME}/octave-4.2.1
export CPP_DIR=${HOME}/cpp
export DLL_DIR=${HOME}/dll
export LIB_DIR=${HOME}/lib
export EXTERNAL_LIBS=${HOME}/externallibs
export ROS_HOME=/opt/ros/kinetic/bin/
export PATH=${ROS_HOME}":"${EXTERNAL_LIBS}":"${LIB_DIR}":"${MINGW_HOME}/bin":"${DLL_DIR}":"${JAVA_HOME}/bin":"${OCTAVE_HOME}/bin":"${PATH}
#export PATH=/usr/lib/gcc/x86_64-linux-gnu/5":"$PATH
export SVG_HOME=${HOME}/visualisation
export REPORTING_HOME=${HOME}/reporting
export CHROME=${HOME}/chrome-linux/google-chrome
export TESTS_CPP_DIR=${HOME}/tests-cpp
export TEST_EXEC_DIR=${HOME}/exec
export ARMADILLO_HOME=${HOME}/armadillo-8.500.1-linux
export MXNET_HOME=${USER_HOME}/incubator-mxnet
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MXNET_HOME}/lib
export DATA_DIR=${HOME}/data
export CLASSIFIER_CPP=${HOME}/build/cpp
export CLASSIFIER_TRAIN=${HOME}/build/train
export CLASSIFIER_TARGET=${HOME}/exec
#CLUSTERER_CPP_DIR=${HOME}/clusterer-cpp
export CLUSTERER_EXEC_DIR=${HOME}/exec
export VERIFICATION_HOME=${HOME}/viewverification
export DISTR_SIM=${HOME}/distributed-simulator
export PSQL=${DISTR_SIM}/PostgreSQLPortable
export SFS=${DISTR_SIM}/SmartFoxServer_2X/SFS2X
export RMI=${DISTR_SIM}/RMIModelServer
export RMI_PATH=${EXTERNAL_LIBS}":"${LIB_DIR}":"${MINGW_HOME}/bin":"${DLL_DIR}":"${JAVA_HOME}/bin":"${OCTAVE_HOME}/bin":"
export WASM_HOME=${HOME}/emam2wasm
export PACMAN_HOME=${HOME}/pacman
export ROS_SIM_HOME=${HOME}/RosSimulationFramework
export ROS_CACHE_TIMEOUT=-1.0
