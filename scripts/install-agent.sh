cd target/agentComponent
# change to local ros location
ROS_HOME="/opt/ros/kinetic"
export ROS_HOME
./compile.sh

rm -rf ../../bin/AgentComponent
cp install/bin/Coordinator_agent_torcsAgent ../../bin/AgentComponent
