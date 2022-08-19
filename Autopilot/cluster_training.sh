#!/bin/zsh

### Job name
#SBATCH -A thes1234
#SBATCH --job-name=training

### Output file
#SBATCH --output=output_%J.txt

### Job Specification
#SBATCH --cpus-per-task=6
### SBATCH --gres=gpu:volta:2
#SBATCH --mem-per-cpu=4096m

#SBATCH --time=100:00:00

### module load cuda
### module load cudnn
### module load nccl

singularity exec --nv /rwthfs/rz/SW/UTIL.common/singularity/in296142 $(pwd)/install_and_run_training.sh -sp
