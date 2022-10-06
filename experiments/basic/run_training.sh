#!/bin/bash

. ./config.sh

trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
sleep 5

cd "${SIMULATOR_PATH}"

if [[ "$*" == *"-sp"* ]]
then
	/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -jar basic-simulator.jar -rl -s "${SCENARIO_PATH}" -d -aE &
	sleep 5
	cd ${PROJECT_ROOT}/${BINARY}
	if [ -e "model" ]
	then
	rm -rf model #remove current model folder
	fi
	cd ${PROJECT_ROOT}/"target/agent/src/de_rwth_montisim_agent_master/cpp" #directory of the new model folder
	cp -r ./model ${PROJECT_ROOT}/${BINARY}

	cd "${PROJECT_ROOT}/${BINARY}"
	cd model/de.rwth.montisim.agent.network.AutopilotQNet
	mv model_0-0000.params model_0_newest-0000.params
	mv model_0-symbol.json model_0_newest-symbol.json
	cd "${PROJECT_ROOT}/${BINARY}"
	./agent -t ${SELF_PLAY_AGENT_EXECUTION_INTERVAL_TRAINING} &
	sleep 2
else
	echo "Scenario Path: ${SCENARIO_PATH}"
	/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -jar basic-simulator.jar -rl -s "${SCENARIO_PATH}" &
fi

cd $PROJECT_ROOT
mkdir -p logs
mkdir -p logs/figures
yes | ./run_agent_training_script.sh | tee logs/logfile
