cd target/affordanceComponent
# change to local ros location
ROS_HOME="/opt/ros/kinetic"
export ROS_HOME
./compile.sh

echo "copy binaries"
rm -rf ../../bin/AffordanceComponent
cp install/bin/Coordinator_cv_affordanceComponent ../../bin/AffordanceComponent
