#!/bin/bash
set -e

# Change EMA_INSTALL_BASE to your desired install directory(must be absolute)
curDir=$(readlink -f `dirname $0`)
EMA_INSTALL_BASE="${curDir}/lib"

if [[ "$EMA_INSTALL_BASE" != /* ]];then
    echo "EMA_INSTALL_BASE can not be a relative path!"
    exit 1
fi


ARMADILLO_INSTALL_DIR="${EMA_INSTALL_BASE}/armadillo"
ARMADILLO_DOWNLOAD_LINK="https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip"

COINCAR_INSTALL_DIR="${EMA_INSTALL_BASE}/coincar"
COINCAR_DOWNLOAD_LINK="https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F19.03.07-RosSimulationFramework&files=RosSimulationFramework.zip"

# install armadillo
[[ "$@" != "-y" ]] && read -p "Installing armadillo into $ARMADILLO_INSTALL_DIR. Abort with ctrl-c or continue with enter"
mkdir -p "$ARMADILLO_INSTALL_DIR/"
wget -O /tmp/download.zip "$ARMADILLO_DOWNLOAD_LINK"
unzip -q /tmp/download.zip -d "$ARMADILLO_INSTALL_DIR/"
echo "export Armadillo_HOME='$ARMADILLO_INSTALL_DIR/armadillo-8.500.1-linux'" >> ~/.bashrc
rm /tmp/download.zip

# install coincar_sim
[[ "$@" != "-y" ]] && read -p "Installing coincar_sim into $COINCAR_INSTALL_DIR. Abort with ctrl-c or continue with enter"
mkdir -p "$COINCAR_INSTALL_DIR/"
wget -O /tmp/download.zip "$COINCAR_DOWNLOAD_LINK"
unzip -q /tmp/download.zip -d "$COINCAR_INSTALL_DIR/"
echo "export COINCAR_SIM_HOME='$COINCAR_INSTALL_DIR/RosSimulationFramework'" >> ~/.bashrc
rm /tmp/download.zip

# add ROS_HOME
echo "export ROS_HOME='/opt/ros/kinetic" >> ~/.bashrc

# compile coincar_sim
cd "$COINCAR_INSTALL_DIR/RosSimulationFramework"
./build.sh
cd -