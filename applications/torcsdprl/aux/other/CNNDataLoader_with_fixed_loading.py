import os
import h5py
import mxnet as mx
import logging
import sys
from mxnet import nd
import numpy as np 

class CNNDataLoader_dpnet:
    _input_names_ = ['data']
    _output_names_ = ['predictions_label']

    def __init__(self):
        self._data_dir = "random/"

    def load_data(self, batch_size):
        train_h5, test_h5 = self.load_h5_files()

        train_data = {}
        data_mean = {}
        data_std = {}

        for input_name in self._input_names_: 
            train_data[input_name] = train_h5[input_name]
            train_dataset = train_h5[input_name]
            train_dataset_shape = train_data[input_name].shape
            # slize_size limits the memory cosumption, by only loading slizes of size <500MB into memory
            slize_size = min(train_dataset_shape[0] - 1, int(500e6 / (train_h5[input_name][0].size * train_h5[input_name][0].itemsize)))
            num_slizes = max(1, int(train_h5[input_name].shape[0] / slize_size))
            mean = np.zeros(train_dataset_shape[1: ])
            std = np.zeros(train_dataset_shape[1: ])
            
            for i in range(int(train_dataset_shape[0] / slize_size)):
                mean += train_dataset[i * slize_size: (i + 1) * slize_size].mean(axis=0) / num_slizes
                std += train_dataset[i * slize_size: (i + 1) * slize_size].std(axis=0) / num_slizes
            if slize_size > train_dataset_shape[0] - 1:
                mean += train_dataset[num_slizes * slize_size: ].mean(axis=0) / (slize_size - num_slizes % slize_size)
                std += train_dataset[num_slizes * slize_size: ].std(axis=0) / (slize_size - num_slizes % slize_size)
            std += 1e-5

            data_mean[input_name + '_'] = nd.array(mean)
            data_std[input_name + '_'] = nd.array(std)

        train_label = {}
        for output_name in self._output_names_:
            train_label[output_name] = train_h5[output_name]

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size)

        test_iter = None

        if test_h5 != None:
            test_data = {}
            for input_name in self._input_names_:
                test_data[input_name] = test_h5[input_name]

            test_label = {}
            for output_name in self._output_names_:
                test_label[output_name] = test_h5[output_name]

            test_iter = mx.io.NDArrayIter(data=test_data,
                                          label=test_label,
                                          batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir + "train.h5"
        test_path = self._data_dir + "test.h5"

        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')

            for input_name in self._input_names_:
                if not input_name in train_h5:
                    logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the dataset "
                                  + "'" + input_name + "'")
                    sys.exit(1)

            for output_name in self._output_names_:
                if not output_name in train_h5:
                    logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the dataset "
                                  + "'" + output_name + "'")
                    sys.exit(1)

            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')

                for input_name in self._input_names_:
                    if not input_name in test_h5:
                        logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the dataset "
                                      + "'" + input_name + "'")
                        sys.exit(1)

                for output_name in self._output_names_:
                    if not output_name in test_h5:
                        logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the dataset "
                                      + "'" + output_name + "'")
                        sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")

            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)
