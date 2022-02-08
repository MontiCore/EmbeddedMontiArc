directory=/home/ClusterID/path/to/autopilot
. $directory/config.sh

cd "target/agent/src/de_rwth_montisim_agent_master/cpp"
python CNNTrainer_de_rwth_montisim_agent_master_qnet.py

cp -r ./model ${PROJECT_ROOT}/${BINARY}
cd "${PROJECT_ROOT}"
