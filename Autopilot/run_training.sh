trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
sleep 2 
oldpath=$(pwd)
. $oldpath/config.sh
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar -rl -s "${SCENARIO_PATH}" & 
cd $oldpath
yes | ./run_agent_training_script.sh

