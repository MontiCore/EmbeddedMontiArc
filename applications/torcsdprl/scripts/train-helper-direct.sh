# helper script, should not be run directly
source /opt/ros/kinetic/setup.sh

cd target/directAgent/src/agent_directAgent/cpp
sh start_training.sh
#xterm -title "TRAIN" -e "sh start_training.sh; bash" &
#cp -r ./model ../../../../model/agentComponent
