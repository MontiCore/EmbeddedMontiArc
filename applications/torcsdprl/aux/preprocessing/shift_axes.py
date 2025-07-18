'''
Mxnet's convention for the convolution input shpe is channels x spatial_dimension.
This differs from the normal convention for images, which is spatial_dimension x channels.
'''
import h5py
from mxnet.ndarray.gen_op import slice_axis
import numpy as np
from tqdm import tqdm

def shift_axes(hdf5_path):
    # to do test set 
    train_file = h5py.File(hdf5_path + "train-tmp.h5", "a")
    train_data = train_file["data"]
    train_dataset_shape = train_data.shape
    train_size = train_dataset_shape[0]

    test_file = h5py.File(hdf5_path + "test-tmp.h5", "a")
    test_data = test_file["data"]
    test_dataset_shape = test_data.shape
    test_size = test_dataset_shape[0]

    updated_train_file = h5py.File(hdf5_path + "train.h5", "a")
    train_tmp = updated_train_file.create_dataset("data", (train_size, 3, 210, 280), dtype="uint8", chunks=True)
    train_lbl = updated_train_file.create_dataset("predictions_label", (train_size, 14), dtype="f", chunks=True)
    updated_test_file = h5py.File(hdf5_path + "test.h5", "a")
    test_tmp = updated_test_file.create_dataset("data", (test_size, 3, 210, 280), dtype="uint8", chunks=True)
    test_lbl = updated_test_file.create_dataset("predictions_label", (test_size, 14), dtype="f", chunks=True)


    # slize_size limits the memory cosumption, by only loading slizes of size <500MB into memory
    slize_size = min(train_size - 1, int(500e6 / (train_data[0].size * train_data[0].itemsize)))
    test_slize_size = min(test_size - 1, int(500e6 / (test_data[0].size * test_data[0].itemsize)))

    print("\nshift axes in training-set")
    with tqdm(total=int(train_size / slize_size)) as bar:
        for i in range(int(train_size / slize_size)):
            train_tmp[i * slize_size: (i + 1) * slize_size] = np.moveaxis(train_data[i * slize_size: (i + 1) * slize_size], [3], [1])
            train_lbl[i * slize_size: (i + 1) * slize_size] = train_file["predictions_label"][i * slize_size: (i + 1) * slize_size]
            bar.update(1)

    print("\nshift axes in test-set")
    with tqdm(total=int(test_size / test_slize_size)) as bar:
        for i in range(int(test_size / test_slize_size)):
            test_tmp[i * test_slize_size: (i + 1) * test_slize_size] = np.moveaxis(test_data[i * test_slize_size: (i + 1) * test_slize_size], [3], [1])
            test_lbl[i * test_slize_size: (i + 1) * test_slize_size] = test_file["predictions_label"][i * test_slize_size: (i + 1) * test_slize_size]
            bar.update(1)

    # to do write in batches
    # updated_train_file["predictions_label"] = train_file["predictions_label"]
    # updated_test_file["predictions_label"] = test_file["predictions_label"]

    #del train_file["data"]
    #train_file["data"] = train_file["tmp_data"]
    #del train_file["tmp_data"]
    
    #del test_file["data"]
    #test_file["data"] = test_file["tmp_data"]
    #del test_file["tmp_data"]
