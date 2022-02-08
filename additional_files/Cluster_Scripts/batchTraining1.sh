#!/bin/zsh

### Job name
#SBATCH --job-name=training

### Output file
#SBATCH --output=output_%J.txt

### Project name
#SBATCH -A thes1234

#SBATCH --time=32:00:00

#SBATCH --mem-per-cpu=1000m

#SBATCH --cpus-per-task=12

###SBATCH --mail-user=max.mustermann@rwth-aachen.de

###SBATCH --mail-type=END

singularity exec /rwthfs/rz/SW/UTIL.common/singularity/RLContainerNew $HOME/Path/to/Clusterfolder/installAndTrain.sh
