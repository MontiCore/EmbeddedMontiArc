trap 'kill $(ps -o pid= --ppid $$)' INT TERM EXIT
roscore &
sleep 5 
oldpath=/home/ClusterID/path/to/autopilot
. $oldpath/config.sh
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar -rl -s "${SCENARIO_PATH}" & 
cd $oldpath
yes | ./run_agent_training_script.sh
sleep 1
