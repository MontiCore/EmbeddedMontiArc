import h5py
import numpy as np
import os

path = os.path.expanduser("~") + "/.mxnet/datasets/voc/train.h5"

with h5py.File(path, 'r') as fh5:
    for key in fh5.keys():
        print(key, len(fh5[key]))
        print(fh5[key].shape)
