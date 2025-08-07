# helper script, should not be run directly
source /opt/ros/kinetic/setup.sh

cd target/agentComponent/src/agent_torcsAgent/cpp
sh start_training.sh
#xterm -title "TRAIN" -e "sh start_training.sh; bash" &
cp -r ./model ../../../../model/agentComponent
