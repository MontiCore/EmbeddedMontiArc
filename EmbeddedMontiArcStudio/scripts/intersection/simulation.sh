ROS_SIM_HOME=/home/alexander/sppSimulator/

echo "Starting simulation..."
cd ../shared
source variables.sh
source /opt/ros/kinetic/setup.sh
cd $ROS_SIM_HOME

echo DISPLAY=$DISPLAY

#Hack to enable graphical output  
export DISPLAY=":0"
./allSteps.sh
