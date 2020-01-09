import mxnet as mx
import logging
import os

from CNNNet_rnnsearch_main_net import Net_0
from CNNNet_rnnsearch_main_net import Net_1
from CNNNet_rnnsearch_main_net import Net_2
from CNNNet_rnnsearch_main_net import Net_3

class CNNCreator_rnnsearch_main_net:
    _model_dir_ = "model/rnnsearch.Network/"
    _model_prefix_ = "model"

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        #self.weight_initializer = mx.init.Uniform(0,1)
        self.networks = {}

    def load(self, context):
        earliestLastEpoch = None

        for i, network in self.networks.items():
            lastEpoch = 0
            param_file = None

            try:
                os.remove(self._model_dir_ + self._model_prefix_ + "_" + str(i) + "_newest-0000.params")
            except OSError:
                pass
            try:
                os.remove(self._model_dir_ + self._model_prefix_ + "_" + str(i) + "_newest-symbol.json")
            except OSError:
                pass

            if os.path.isdir(self._model_dir_):
                for file in os.listdir(self._model_dir_):
                    if ".params" in file and self._model_prefix_ + "_" + str(i) in file:
                        epochStr = file.replace(".params","").replace(self._model_prefix_ + "_" + str(i) + "-","")
                        epoch = int(epochStr)
                        if epoch > lastEpoch:
                            lastEpoch = epoch
                            param_file = file
            if param_file is None:
                earliestLastEpoch = 0
            else:
                logging.info("Loading checkpoint: " + param_file)
                network.load_parameters(self._model_dir_ + param_file)

                if earliestLastEpoch == None or lastEpoch < earliestLastEpoch:
                    earliestLastEpoch = lastEpoch + 1

        return earliestLastEpoch

    def construct(self, context, data_mean=None, data_std=None):
        self.networks[0] = Net_0(data_mean=data_mean, data_std=data_std)
        self.networks[0].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[0].hybridize()
        self.networks[0](mx.nd.zeros((1, 30,), ctx=context), mx.nd.zeros((1, 2,1000,), ctx=context))
        self.networks[1] = Net_1(data_mean=data_mean, data_std=data_std)
        self.networks[1].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[1].hybridize()
        self.networks[1](mx.nd.zeros((1, 1,), ctx=context))
        self.networks[2] = Net_2(data_mean=data_mean, data_std=data_std)
        self.networks[2].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[2].hybridize()
        self.networks[2](mx.nd.zeros((1, 2,1000,), ctx=context))
        self.networks[3] = Net_3(data_mean=data_mean, data_std=data_std)
        self.networks[3].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[3].hybridize()
        self.networks[3](mx.nd.zeros((1, 1,1000,), ctx=context), mx.nd.zeros((1, 30,1000,), ctx=context), mx.nd.zeros((1, 1,), ctx=context))

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        #for i, network in self.networks.items():
            #network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)
