#!/usr/local_rwth/bin/zsh

### #SBATCH directives need to be in the first part of the jobscript
#SBATCH --time=5
#SBATCH --gres=gpu:1
#SBATCH --output=log_itc.log

### your code goes here, the second part of the jobscript

source ../setup_itc.sh

rm -rf target
java -jar $EMADL_GEN_PATH -m src/emadl/models/ -r cNNSegment.Connector -o target -b GLUON -p $PYTHONPATH -c n
