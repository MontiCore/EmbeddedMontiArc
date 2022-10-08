#!/bin/bash

PROJECT_ROOT="$(pwd)"
BINARY="target/bin"
GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.1.4-SNAPSHOT-jar-with-dependencies.jar"
AGENT_BUILD="target/agent/build"
SIMULATOR_PATH="/builds/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/dev/basic-simulator/install"
SCENARIO_PATH="$PROJECT_ROOT/scenario_file.json"
SAVE_FOLDER="/builds/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/dev/coopmontisimautopilot/saved_autopilots"
SELF_PLAY_AGENT_EXECUTION_INTERVAL_TRAINING=6 # in milliseconds. Adjust for your system. The lower, the better. Only applies to training. You should only see a handful of "Cant keep up" messages as a maximum in the output
SELF_PLAY_AGENT_EXECUTION_INTERVAL_RUNNING=100 # in milliseconds. Set equal to the simulation step in the scenario file. Only applies to the execution


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
	/usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java -jar basic-simulator.jar -rl -s "${SCENARIO_PATH}" &
fi

cd $PROJECT_ROOT
mkdir -p logs
mkdir -p logs/figures
yes | ./run_agent_training_script.sh | tee logs/logfile
