CONTAINER=aux/singularity/gym-container/RL-Torcs-Container

./scripts/run-environment.sh
sleep 4


singularity exec $CONTAINER bash scripts/train-helper-direct.sh

#singularity shell $CONTAINER 
