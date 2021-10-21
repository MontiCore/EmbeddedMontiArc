CONTAINER=aux/singularity/gym-container/RL-Torcs-Container

./scripts/run-environment.sh
sleep 4

#xterm -title "AFFORDANCE COMPONENT" -e "./bin/AffordanceComponent" &
xterm -title "AFFORDANCE COMPONENT" -e "python affordance-component.py" &

#xterm -title "TRAINER" -e "singularity exec $CONTAINER bash scripts/train-helper.sh"
singularity exec $CONTAINER bash scripts/train-helper.sh

#singularity shell $CONTAINER 
