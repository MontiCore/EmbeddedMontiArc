#!/bin/bash

echo "Trying to update the agent ...."

directory=$(pwd)
. $directory/config.sh
cd $directory


cd ${PROJECT_ROOT}/${BINARY}
rm -rf model #remove current model folder

cd ${PROJECT_ROOT}/"generator-target/agent/src/${NAME_OF_AGENT}_agent_master/cpp" #directory of the new model folder
cp -r ./model ${PROJECT_ROOT}/${BINARY} #copy model folder to directory that is used by agent binary
cd ${PROJECT_ROOT}/${BINARY}/"model/${NAME_OF_AGENT_U}Agent"
		

for entry in ${PROJECT_ROOT}/${BINARY}/"model/${NAME_OF_AGENT_U}Agent"/* #find latest training session
do
  dir_name="$entry"
done
cd $dir_name

snapshot=$1 #load the snapshot interval
current_highest=0
	

for i in {0..1000} #find latest snapshot parameters
do
  test_counter=$((snapshot * i))
  if [[ -e "${NAME_OF_AGENT_U}Agent_qnet-ep$test_counter-0000.params" && $test_counter -gt $current_highest ]]
  then
    current_highest=$test_counter #denotes latest snapshot
  fi
done

params_name="${NAME_OF_AGENT_U}Agent_qnet-ep$current_highest-0000.params" #extraxt newest snapshot parameters
json_name="${NAME_OF_AGENT_U}Agent_qnet-ep$current_highest-symbol.json"

cp $params_name ${PROJECT_ROOT}/${BINARY}/"model/${NAME_OF_AGENT_U}.agent.network.${NAME_OF_AGENT_U}QNet" #copy snapshot parameters to folder that is used by the agent binary
cp $json_name ${PROJECT_ROOT}/${BINARY}/"model/${NAME_OF_AGENT}.agent.network.${NAME_OF_AGENT_U}QNet"
cd ${PROJECT_ROOT}/${BINARY}/"model/${NAME_OF_AGENT}.agent.network.${NAME_OF_AGENT_U}QNet"

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