import caffe
from caffe.proto import caffe_pb2
import h5py
import numpy as np
import plyvel
from sklearn.model_selection import train_test_split
from tqdm import tqdm


def leveldb_to_hdf5(level_db_path, hdf5_path, relative_test_size, conversion_batch_size):
    BATCH_SIZE = conversion_batch_size
    size = 484814
    train_size = int(np.ceil(size * (1 - relative_test_size)))
    test_size = int(np.ceil(size * relative_test_size))

    db = plyvel.DB(level_db_path, paranoid_checks=True, create_if_missing=False)
    datum = caffe_pb2.Datum()

    train_images = []
    train_indicators = []
    train_file_idx = 1
    test_images = []
    test_indicators = []
    test_file_idx = 1

    train_file = h5py.File(hdf5_path + "train-tmp.h5", 'a')
    test_file = h5py.File(hdf5_path + "test-tmp.h5", 'a')
    training_data = train_file.create_dataset('data', (train_size, 210, 280, 3) , dtype='uint8', chunks=True) 
    training_labels = train_file.create_dataset('predictions_label', (train_size, 14), dtype='f', chunks=True)
    testing_data = test_file.create_dataset('data', (test_size, 210, 280, 3), dtype='uint8', chunks=True) 
    testing_labels = test_file.create_dataset('predictions_label', (test_size, 14), dtype='f', chunks=True)

    keys = range(1, size)
    train_keys, test_keys = train_test_split(keys, test_size=relative_test_size)
    print(str(len(train_keys)) + " samples for training")
    print(str(len(test_keys)) + " samples for testing")

    print("\nconvert levelDB to hdf5:")
    with tqdm(total=size) as bar:
        for key, value in db:
            key_as_int = int(float(key))

            datum = datum.FromString(value)
            indicators = np.array(datum.float_data, dtype='f')

            image_data = caffe.io.datum_to_array(datum) 
            image_data = np.transpose(image_data, (1, 2, 0))
            image_data = image_data[:, :, ::-1]

            if key_as_int in train_keys:
                train_images.append(image_data)
                train_indicators.append(indicators)
                if len(train_images) >= BATCH_SIZE:
                    pos = (train_file_idx - 1) * BATCH_SIZE
                    training_data[pos: pos + BATCH_SIZE, :, :, :] = train_images
                    training_labels[pos: pos + BATCH_SIZE, :] = train_indicators
                    train_images = []
                    train_indicators = []
                    train_file_idx += 1

            elif key_as_int in test_keys:
                test_images.append(image_data)
                test_indicators.append(indicators)
                if len(test_images) >= BATCH_SIZE:
                    pos = (test_file_idx - 1) * BATCH_SIZE
                    testing_data[pos: pos + BATCH_SIZE, :, :, :] = test_images
                    testing_labels[pos: pos + BATCH_SIZE, :] = test_indicators
                    test_images = []
                    test_indicators = []
                    test_file_idx += 1

            bar.update(1)
        # dumping last batch
        training_data[BATCH_SIZE * (train_file_idx - 1): BATCH_SIZE * (train_file_idx - 1) + len(train_images), :, :, :] = train_images
        training_labels[BATCH_SIZE * (train_file_idx - 1): BATCH_SIZE * (train_file_idx - 1) + len(train_indicators), :] = train_indicators
        testing_data[BATCH_SIZE * (test_file_idx - 1): BATCH_SIZE * (test_file_idx - 1) + len(test_images), :, :, :] = test_images
        testing_labels[BATCH_SIZE * (test_file_idx - 1): BATCH_SIZE * (test_file_idx - 1) + len(test_indicators), :] = test_indicators
        






