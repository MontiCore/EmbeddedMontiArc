import os
import h5py
import mxnet as mx
import logging
import sys
import numpy as np
import cv2
import importlib
from mxnet import nd

class CNNDataLoader_RNNencdec:
    _input_names_ = ['source']
    _output_names_ = ['target_0_label','target_1_label','target_2_label','target_3_label','target_4_label','target_5_label','target_6_label','target_7_label','target_8_label','target_9_label','target_10_label','target_11_label','target_12_label','target_13_label','target_14_label','target_15_label','target_16_label','target_17_label','target_18_label','target_19_label','target_20_label','target_21_label','target_22_label','target_23_label','target_24_label','target_25_label','target_26_label','target_27_label','target_28_label','target_29_label']

    def __init__(self):
        self._data_dir = "data/RNNencdec/"

    def load_data(self, batch_size):
        train_h5, test_h5 = self.load_h5_files()

        train_data = {}
        data_mean = {}
        data_std = {}
        train_images = {}

        for input_name in self._input_names_:
            train_data[input_name] = train_h5[input_name]
            data_mean[input_name + '_'] = nd.array(train_h5[input_name][:].mean(axis=0))
            data_std[input_name + '_'] = nd.array(train_h5[input_name][:].std(axis=0) + 1e-5)

            if 'images' in train_h5:
                train_images = train_h5['images']

        train_label = {}
        index = 0
        for output_name in self._output_names_:
            train_label[index] = train_h5[output_name]
            index += 1

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size)

        test_iter = None

        if test_h5 != None:
            test_data = {}
            test_images = {}
            for input_name in self._input_names_:
                test_data[input_name] = test_h5[input_name]

                if 'images' in test_h5:
                    test_images = test_h5['images']

            test_label = {}
            index = 0
            for output_name in self._output_names_:
                test_label[index] = test_h5[output_name]
                index += 1

            test_iter = mx.io.NDArrayIter(data=test_data,
                                          label=test_label,
                                          batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std, train_images, test_images

    def load_preprocessed_data(self, batch_size, preproc_lib):
        train_h5, test_h5 = self.load_h5_files()

        wrapper = importlib.import_module(preproc_lib)
        instance = getattr(wrapper, preproc_lib)()
        instance.init()
        lib_head, _sep, tail = preproc_lib.rpartition('_')
        inp = getattr(wrapper, lib_head + "_input")()

        train_data = {}
        train_label = {}
        data_mean = {}
        data_std = {}

        shape_output = self.preprocess_data(instance, inp, 0, train_h5)
        train_len = len(train_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (train_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_data[input_name] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (train_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(train_len):
            output = self.preprocess_data(instance, inp, i, train_h5)
            for input_name in self._input_names_:
                train_data[input_name][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                train_label[output_name][i] = getattr(shape_output, output_name + "_out")

        for input_name in self._input_names_:
            data_mean[input_name + '_'] = nd.array(train_data[input_name][:].mean(axis=0))
            data_std[input_name + '_'] = nd.array(train_data[input_name][:].asnumpy().std(axis=0) + 1e-5)

        if 'images' in train_h5:
            train_images = train_h5['images']

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size)

        test_data = {}
        test_label = {}

        shape_output = self.preprocess_data(instance, inp, 0, test_h5)
        test_len = len(test_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (test_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_data[input_name] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (test_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(test_len):
            output = self.preprocess_data(instance, inp, i, test_h5)
            for input_name in self._input_names_:
                test_data[input_name][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                test_label[output_name][i] = getattr(shape_output, output_name + "_out")

        if 'images' in test_h5:
            test_images = test_h5['images']

        test_iter = mx.io.NDArrayIter(data=test_data,
                                       label=test_label,
                                       batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std, train_images, test_images

    def preprocess_data(self, instance_wrapper, input_wrapper, index, data_h5):
        for input_name in self._input_names_:
            data = data_h5[input_name][0]
            attr = getattr(input_wrapper, input_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, input_name, data)
        for output_name in self._output_names_:
            data = data_h5[output_name][0]
            attr = getattr(input_wrapper, output_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, output_name, data)
        return instance_wrapper.execute(input_wrapper)

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
