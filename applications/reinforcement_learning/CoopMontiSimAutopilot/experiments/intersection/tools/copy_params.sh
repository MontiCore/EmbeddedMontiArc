cd ../target/agent/src/de_rwth_montisim_agent_master/cpp/model/de.rwth.montisim.agent.network.AutopilotQNet/AutopilotAgent
LATEST=$(ls -t | head -n 1)
cd ../../../../../../..
cp agent/src/de_rwth_montisim_agent_master/cpp/model/de.rwth.montisim.agent.network.AutopilotQNet/AutopiletAgent/${LATEST}/best_network-0000.params bin/model/de.rwth.montisim.agent.network.AutopilotQNet/model_0_newest-0000.params
cp agent/src/de_rwth_montisim_agent_master/cpp/model/de.rwth.montisim.agent.network.AutopilotQNet/AutopiletAgent/${LATEST}/best_network-symbol.json bin/model/de.rwth.montisim.agent.network.AutopilotQNet/model_0_newest-symbol.json 