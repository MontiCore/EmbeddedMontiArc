#!/bin/bash

trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
. ./config.sh

sleep 2
oldpath=$(pwd)
. $oldpath/config.sh
cd "${BINARY}"
if [ -d "model_old" ]; then
	/bin/echo "model_old exists"
	if [ -d "model" ]; then
		/bin/echo "removing model"
		rm -r model
	fi
else
	mv model model_old
fi
cp -R ../agent/src/de_rwth_montisim_agent_master/cpp/model/ .
cd model/de.rwth.montisim.agent.network.AutopilotQNet
mv model_0_newest-0000.params model_0_newest-0000_OLD.params
mv model_0_newest-symbol.json model_0_newest-symbol_OLD.json
mv model_0-0000.params model_0_newest-0000.params
mv model_0-symbol.json model_0_newest-symbol.json

cd "${oldpath}"
cd "${BINARY}"
./agent -t ${SELF_PLAY_AGENT_EXECUTION_INTERVAL_RUNNING} &
sleep 2
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar
cd $oldpath
