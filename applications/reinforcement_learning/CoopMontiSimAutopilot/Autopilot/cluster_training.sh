#!/bin/zsh

### Job name
#SBATCH -A thes1234
#SBATCH --job-name=training

### Output file
#SBATCH --output=output_%J.txt

### Job Specification
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=2048m

### #SBATCH --gres=gpu:volta:2

#SBATCH --time=12:00:00

### module load cuda
### module load cudnn
### module load nccl

singularity exec /rwthfs/rz/SW/UTIL.common/singularity/RLContainerCPU $(pwd)/install_and_run_training.sh -sp
