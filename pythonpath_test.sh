#!/usr/bin/env python3
source ./setup_itc.sh

MY_PATH=$(which python3)
echo $MY_PATH


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