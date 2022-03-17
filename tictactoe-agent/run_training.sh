. config.sh

savepath=$(pwd)

cd ${PROJECT_ROOT}/${BINARY}
if [ -e "model" ]
then
  rm -rf model
  
  cd ${PROJECT_ROOT}/"generator-target/agent/src/tictactoe_agent_master/cpp" #directory of new model folder
  cp -r ./model ${PROJECT_ROOT}/${BINARY} 

  cd "${PROJECT_ROOT}/${BINARY}"
  cd model/tictactoe.agent.network.TicTacToeQNet
  mv model_0-0000.params model_0_newest-0000.params
  mv model_0-symbol.json model_0_newest-symbol.json

  cd "${PROJECT_ROOT}/${BINARY}"
  ./agent -executeOnDemand &
  sleep 2

fi

cd $savepath

cd "generator-target/agent/src/tictactoe_agent_master/cpp"

python2.7 CNNTrainer_tictactoe_agent_master_dqn.py
cp -r ./model ${PROJECT_ROOT}/${BINARY}

RUNNING_AGENT=$(pgrep -f ./agent) #abort currently running agent binary

if [ -z "$RUNNING_AGENT" ]
then
      :
else
      kill $RUNNING_AGENT
fi