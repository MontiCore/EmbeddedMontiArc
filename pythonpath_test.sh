#!/usr/bin/env python3

which python
python --version
python3 --version
alias python=python3
python -c "import sys; print(sys.version)"
export PYTHONPATH=$PYTHONPATH:/usr/bin/python3
which python

python3 -c "import mxnet as mx
def gpu_device(gpu_number=0):
    try:
        _ = mx.nd.array([1, 2, 3], ctx=mx.gpu(gpu_number))
    except mx.MXNetError:
        return None
    return mx.gpu(gpu_number)

if not gpu_device:
    print(\"gpu not supported\")
"