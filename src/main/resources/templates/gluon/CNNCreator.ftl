import mxnet as mx
import logging
import os
from CNNNet_${tc.fullArchitectureName} import Net

class ${tc.fileNameWithoutEnding}:
    _model_dir_ = "model/${tc.componentName}/"
    _model_prefix_ = "model"
    _input_shapes_ = [<#list tc.architecture.inputs as input>(${tc.join(input.definition.type.dimensions, ",")},)</#list>]

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        self.net = None

    def get_input_shapes(self):
        return self._input_shapes_

    def load(self, context):
        lastEpoch = 0
        param_file = None

        try:
            os.remove(self._model_dir_ + self._model_prefix_ + "_newest-0000.params")
        except OSError:
            pass
        try:
            os.remove(self._model_dir_ + self._model_prefix_ + "_newest-symbol.json")
        except OSError:
            pass

        if os.path.isdir(self._model_dir_):
            for file in os.listdir(self._model_dir_):
                if ".params" in file and self._model_prefix_ in file:
                    epochStr = file.replace(".params","").replace(self._model_prefix_ + "-","")
                    epoch = int(epochStr)
                    if epoch > lastEpoch:
                        lastEpoch = epoch
                        param_file = file
        if param_file is None:
            return 0
        else:
            logging.info("Loading checkpoint: " + param_file)
            self.net.load_parameters(self._model_dir_ + param_file)
            return lastEpoch


    def construct(self, context, data_mean=None, data_std=None):
        self.net = Net(data_mean=data_mean, data_std=data_std)
        self.net.collect_params().initialize(self.weight_initializer, ctx=context)
        self.net.hybridize()
        self.net(mx.nd.zeros((1,)+self._input_shapes_[0], ctx=context))

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        self.net.export(self._model_dir_ + self._model_prefix_, epoch=0)
