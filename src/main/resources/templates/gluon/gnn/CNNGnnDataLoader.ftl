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
