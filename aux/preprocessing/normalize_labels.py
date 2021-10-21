import numpy as np
import h5py
import tqdm

def normalize_dataset_labels(hdf5_path, label_range):
    train_data = h5py.File(hdf5_path + "train.h5", "a")
    test_data = h5py.File(hdf5_path + "test.h5", "a")
    train_labels = train_data["predictions_label"]
    test_labels = test_data["predictions_label"]

    print("\nnormalize train labels")
    print("this may take a minute")
    for i, label in enumerate(train_labels):
        train_labels[i] = normalize_label(label, label_range)

    print("normalize test labels")
    for i, label in enumerate(test_labels):
        test_labels[i] = normalize_label(label, label_range)


def normalize_label(label, label_range):
    # normalizes labels to range [0, 1]
    return np.array([normalize_value(value, label_range[i]) for i, value in enumerate(label)])

def normalize_value(value, label_range):
    # the actual value may fall outside of the expected range
    # the normalized value on the other hand has a hard range constraint
    normalized_value = (value - label_range[0]) / (label_range[1] - label_range[0])
    return min(1, max(0, normalized_value))

