import os
import h5py
import mxnet as mx
import logging
import sys

class torcs_agent_torcsAgent_dqnDataLoader:
    _input_names_ = ['state']
    _output_names_ = ['qvalues_label']

    def __init__(self):
        self._data_dir = "data/"

    def load_data(self, batch_size):
        train_h5, test_h5 = self.load_h5_files()

        data_mean = train_h5[self._input_names_[0]][:].mean(axis=0)
        data_std = train_h5[self._input_names_[0]][:].std(axis=0) + 1e-5

        train_iter = mx.io.NDArrayIter(train_h5[self._input_names_[0]],
                                       train_h5[self._output_names_[0]],
                                       batch_size=batch_size,
                                       data_name=self._input_names_[0],
                                       label_name=self._output_names_[0])
        test_iter = None
        if test_h5 != None:
            test_iter = mx.io.NDArrayIter(test_h5[self._input_names_[0]],
                                          test_h5[self._output_names_[0]],
                                          batch_size=batch_size,
                                          data_name=self._input_names_[0],
                                          label_name=self._output_names_[0])
        return train_iter, test_iter, data_mean, data_std

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir + "train.h5"
        test_path = self._data_dir + "test.h5"
        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')
            if not (self._input_names_[0] in train_h5 and self._output_names_[0] in train_h5):
                logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the datasets: "
                              + "'" + self._input_names_[0] + "', '" + self._output_names_[0] + "'")
                sys.exit(1)
            test_iter = None
            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')
                if not (self._input_names_[0] in test_h5 and self._output_names_[0] in test_h5):
                    logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the datasets: "
                                  + "'" + self._input_names_[0] + "', '" + self._output_names_[0] + "'")
                    sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")
            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)