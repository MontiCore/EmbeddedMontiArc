<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import os
import h5py
import mxnet as mx
import logging
import sys
import numpy as np
import importlib
<#if tc.architecture.useDgl>
import dgl
from dgl.data.utils import load_graphs
</#if>
from mxnet import nd
# TODO Class name
class CNNGnnDataLoader:
    _input_names_ = [<#list tc.architectureInputs as inputName><#if inputName?index == tc.architectureInputs?seq_index_of(inputName)>'${inputName?keep_before_last("_")}'<#sep>, </#if></#list>]
    _output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "label'")}]

    def __init__(self):
        self._data_dir = "${tc.dataPath}/"

    def load_data(self):
        data_h5 = self.load_h5_files()

        graph = None
        data = {}
        data_mean = {}
        data_std = {}

        for input_name in self._input_names_:
            if input_name in data_h5:
                # TODO implement function for batching, as used in advanced GNN Papers
                data["input"+str(self._input_names_.index(input_name))] = data_h5[input_name]
                dataset = data_h5[input_name]
                dataset_shape = data["input"+str(self._input_names_.index(input_name))].shape
                mean = np.zeros(dataset_shape[1: ])
                std = np.zeros(dataset_shape[1: ])
                mean += dataset[0].mean(axis=0)
                std += dataset[0].std(axis=0)
                std += 1e-5
                data_mean[input_name + '_'] = nd.array(mean)
                data_std[input_name + '_'] = nd.array(std)
            else:
<#if !(tc.architecture.useDgl)>
                logging.error("The HDF5 file has to contain the dataset "
                + "'" + input_name + "', if the input is a graph use generator flag -useDgl y")
                sys.exit(1)
</#if>
                graph, _ = load_graphs(self._data_dir + input_name)
                graph = graph[0]

        label = {}
        index = 0
        for output_name in self._output_names_:
            label[index] = data_h5[output_name]
            index += 1

        data = mx.io.NDArrayIter(data=data, label=label, batch_size=1)
        return data, graph, data_mean, data_std

    # TODO Update this
    def load_preprocessed_data(self, batch_size, preproc_lib, shuffle=False):
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
        train_images = {}

        shape_output = self.preprocess_data(instance, inp, 0, train_h5)
        train_len = len(train_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (train_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_data["input"+str(self._input_names_.index(input_name))] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (train_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (train_len, 1)
            train_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(train_len):
            output = self.preprocess_data(instance, inp, i, train_h5)
            for input_name in self._input_names_:
                train_data["input"+str(self._input_names_.index(input_name))][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                train_label[output_name][i] = getattr(shape_output, output_name + "_out")

        for input_name in self._input_names_:
            data_mean[input_name + '_'] = nd.array(train_data["input"+str(self._input_names_.index(input_name))][:].mean(axis=0))
            data_std[input_name + '_'] = nd.array(train_data["input"+str(self._input_names_.index(input_name))][:].asnumpy().std(axis=0) + 1e-5)

        if 'images' in train_h5:
            train_images = train_h5['images']

        train_iter = mx.io.NDArrayIter(data=train_data,
                                       label=train_label,
                                       batch_size=batch_size,
                                       shuffle=shuffle)

        test_data = {}
        test_label = {}

        shape_output = self.preprocess_data(instance, inp, 0, test_h5)
        test_len = len(test_h5[self._input_names_[0]])

        for input_name in self._input_names_:
            if type(getattr(shape_output, input_name + "_out")) == np.ndarray:
                cur_shape = (test_len,) + getattr(shape_output, input_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_data["input"+str(self._input_names_.index(input_name))] = mx.nd.zeros(cur_shape)
        for output_name in self._output_names_:
            if type(getattr(shape_output, output_name + "_out")) == nd.array:
                cur_shape = (test_len,) + getattr(shape_output, output_name + "_out").shape
            else:
                cur_shape = (test_len, 1)
            test_label[output_name] = mx.nd.zeros(cur_shape)

        for i in range(test_len):
            output = self.preprocess_data(instance, inp, i, test_h5)
            for input_name in self._input_names_:
                test_data["input"+str(self._input_names_.index(input_name))][i] = getattr(output, input_name + "_out")
            for output_name in self._output_names_:
                test_label[output_name][i] = getattr(shape_output, output_name + "_out")

        test_images = {}
        if 'images' in test_h5:
            test_images = test_h5['images']

        test_iter = mx.io.NDArrayIter(data=test_data,
                                       label=test_label,
                                       batch_size=batch_size)

        return train_iter, test_iter, data_mean, data_std, train_images, test_images

    def preprocess_data(self, instance_wrapper, input_wrapper, index, data_h5):
        for input_name in self._input_names_:
            data = data_h5[input_name][index]
            attr = getattr(input_wrapper, input_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, input_name, data)
        for output_name in self._output_names_:
            data = data_h5[output_name][index]
            attr = getattr(input_wrapper, output_name)
            if (type(data)) == np.ndarray:
                data = np.asfortranarray(data).astype(attr.dtype)
            else:
                data = type(attr)(data)
            setattr(input_wrapper, output_name, data)
        return instance_wrapper.execute(input_wrapper)

    def load_h5_files(self):
        data_h5 = None
        data_path = self._data_dir + "train.h5"

        if os.path.isfile(data_path):
            data_h5 = h5py.File(data_path, 'r')

            for output_name in self._output_names_:
                if not output_name in data_h5:
                    logging.error("The HDF5 file '" + os.path.abspath(data_path) + "' has to contain the dataset "
                                  + "'" + output_name + "'")
                    sys.exit(1)

            return data_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(data_path) + "' does not exist.")
            sys.exit(1)
