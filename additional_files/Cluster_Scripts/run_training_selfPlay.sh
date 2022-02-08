trap 'kill $(ps -o pid= --ppid $$)' INT TERM EXIT
roscore &
sleep 5
oldpath=/home/ClusterID/path/to/autopilot
. $oldpath/config.sh
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar -rl -s  "${SCENARIO_PATH}" & 
sleep 5
cd ${PROJECT_ROOT}/${BINARY}
if [ -e "model" ]
then
  rm -rf model #remove current model folder
fi
cd ${PROJECT_ROOT}/"target/agent/src/de_rwth_montisim_agent_master/cpp" #directory of the new model folder
cp -r ./model ${PROJECT_ROOT}/${BINARY}
#cd ${PROJECT_ROOT}/${BINARY}/"model/AutopilotAgent"

cd "${PROJECT_ROOT}/${BINARY}"
cd model/de.rwth.montisim.agent.network.AutopilotQNet
mv model_0-0000.params model_0_newest-0000.params
mv model_0-symbol.json model_0_newest-symbol.json
cd "${PROJECT_ROOT}/${BINARY}"
./agent -t 50
sleep 2
cd $oldpath
yes | ./run_agent_training_script.sh
sleep 1

