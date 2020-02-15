#!/usr/local_rwth/bin/zsh

### #SBATCH directives need to be in the first part of the jobscript
#SBATCH --time=1
#SBATCH --gres=gpu:1
#SBATCH --output=output_test.txt

### your code goes here, the second part of the jobscript

source ./setup_itc.sh


$PYTHONPATH -c "import mxnet as mx
def gpu_device(gpu_number=0):
    try:
        _ = mx.nd.array([1, 2, 3], ctx=mx.gpu(gpu_number))
    except mx.MXNetError:
        return None
    return mx.gpu(gpu_number)

if not gpu_device:
    print(\"gpu not supported\")
else:
    print(\"gpu available\")
"