CONTAINER=aux/singularity/gym-container/RL-Torcs-Container

./scripts/run-environment.sh
sleep 4

#xterm -title "AFFORDANCE COMPONENT" -e "./bin/AffordanceComponent" &
xterm -title "AFFORDANCE COMPONENT" -e "python affordance-component.py" &

./bin/AgentComponent
#singularity shell $CONTAINER 
