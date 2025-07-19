import numpy as np
import mxnet as mx
from mxnet import nd
import h5py
from PIL import Image
from tqdm import tqdm
import json


f = open("debugging_dataset-conifg.json") 
config = json.loads(f.read())
debuggin_dataset_path = config["debugging_dataset_path"]
split = config["split"]
f.close()
train_h5 = h5py.File(debuggin_dataset_path+ "train.h5")
train_size = train_h5["data"].shape[0]
_input_names_ = ['data']
train_data = {} 
data_mean = {}
data_std = {}

for input_name in _input_names_: # self.
    train_data[input_name] = train_h5[input_name]
    train_dataset = train_h5[input_name]
    train_dataset_shape = train_data[input_name].shape
    # slize_size limits the memory cosumption, by only loading slizes of size <500MB into memory
    slize_size = min(train_dataset_shape[0] - 1, int(500e6 / (train_h5[input_name][0].size * train_h5[input_name][0].itemsize)))
    num_slizes = max(1, int(train_h5[input_name].shape[0] / slize_size))
    mean = np.zeros(train_dataset_shape[1: ])
    std = np.zeros(train_dataset_shape[1: ])
    
    with tqdm(total=int(train_dataset_shape[0] / slize_size)) as bar:
        for i in range(int(train_dataset_shape[0] / slize_size)):
            mean += train_dataset[i * slize_size: (i + 1) * slize_size].mean(axis=0) / num_slizes
            std += train_dataset[i * slize_size: (i + 1) * slize_size].std(axis=0) / num_slizes
            bar.update(1)
    if slize_size > train_dataset_shape[0] - 1:
        mean += train_dataset[num_slizes * slize_size: ].mean(axis=0) / (slize_size - num_slizes % slize_size)
        std += train_dataset[num_slizes * slize_size: ].std(axis=0) / (slize_size - num_slizes % slize_size)
    std += 1e-5

    data_mean[input_name + '_'] = nd.array(mean)
    data_std[input_name + '_'] = nd.array(std)

    mean = np.moveaxis(mean, [0], [2])
    std = np.moveaxis(std, [0], [2])
    img = Image.fromarray(mean.astype("uint8"), 'RGB')
    img.save("mean.png")
    img.show()
    img = Image.fromarray(std.astype("uint8"), 'RGB')
    img.show()
    img.save("std-dev.png")