#!/bin/bash

trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
. ./config.sh

sleep 2
oldpath=$(pwd)
. $oldpath/config.sh

# copy file if arguments contains the flag
if [[ "$*" == *"-auto"* ]]
then
	cd "${BINARY}"
	if ! [ -d "model_old" ]
	then
		cp model model_old -r
	fi

	for entry in ${PROJECT_ROOT}/${BINARY}/"model/AutopilotAgent"/* #find latest training session
	do
		dir_name="$entry"
	done

	cd model/de.rwth.montisim.agent.network.AutopilotQNet
	rm * # delete current files

	# copy best network files
	cp ${dir_name}/best_network-symbol.json model_0_newest-symbol.json
	cp ${dir_name}/best_network-0000.params model_0_newest-0000.params
fi

cd ${oldpath}
cd "${BINARY}"
./agent -t ${SELF_PLAY_AGENT_EXECUTION_INTERVAL_RUNNING} &
sleep 2
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar
cd $oldpath
