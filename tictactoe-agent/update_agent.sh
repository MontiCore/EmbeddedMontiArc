#!/bin/bash

echo "Trying to update the agent ...."

directory=$(pwd)
. $directory/config.sh
cd $directory


cd ${PROJECT_ROOT}/${BINARY}
rm -rf model #remove current model folder

cd ${PROJECT_ROOT}/"generator-target/agent/src/tictactoe_agent_master/cpp" #directory of the new model folder
cp -r ./model ${PROJECT_ROOT}/${BINARY} #copy model folder to directory that is used by agent binary
cd ${PROJECT_ROOT}/${BINARY}/"model/TicTacToeAgent"
		

for entry in ${PROJECT_ROOT}/${BINARY}/"model/TicTacToeAgent"/* #find latest training session
do
  dir_name="$entry"
done
cd $dir_name

snapshot=$1 #load the snapshot interval
current_highest=0
	

for i in {0..1000} #find latest snapshot parameters
do
  test_counter=$((snapshot * i))
  if [[ -e "TicTacToeAgent_qnet-ep$test_counter-0000.params" && $test_counter -gt $current_highest ]]
  then
    current_highest=$test_counter #denotes latest snapshot
  fi
done

params_name="TicTacToeAgent_qnet-ep$current_highest-0000.params" #extraxt newest snapshot parameters
json_name="TicTacToeAgent_qnet-ep$current_highest-symbol.json"

cp $params_name ${PROJECT_ROOT}/${BINARY}/"model/tictactoe.agent.network.TicTacToeQNet" #copy snapshot parameters to folder that is used by the agent binary
cp $json_name ${PROJECT_ROOT}/${BINARY}/"model/tictactoe.agent.network.TicTacToeQNet"
cd ${PROJECT_ROOT}/${BINARY}/"model/tictactoe.agent.network.TicTacToeQNet"

rm -rf model_0_newest-0000.params
rm -rf model_0_newest-symbol.json
mv $params_name model_0_newest-0000.params #rename new snapshot file to be used by the agent executable
mv $json_name model_0_newest-symbol.json
	
RUNNING_AGENT=$(pgrep -f ./agent) #abort currently running agent binary
kill $RUNNING_AGENT

#restart agent with updated network weights
cd ${PROJECT_ROOT}/${BINARY}
./agent -executeOnDemand &
sleep 2
cd ${PROJECT_ROOT}

echo "agent was updated ...."