import mxnet as mx
import logging
import os

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
from CNNNet_${tc.fullArchitectureName} import Net_${stream?index}
</#if>
</#list>

<#list tc.architecture.unrolls as unroll>
<#list unroll.getBodiesForAllTimesteps() as body>
<#if body.isTrainable()>
from CNNNet_${tc.fullArchitectureName} import Net_${tc.architecture.streams?size + body?index}
</#if>
</#list>
</#list>

class ${tc.fileNameWithoutEnding}:
    _model_dir_ = "model/${tc.componentName}/"
    _model_prefix_ = "model"

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
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
                    earliestLastEpoch = lastEpoch

        return earliestLastEpoch

    def construct(self, context, data_mean=None, data_std=None):
<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
        self.networks[${stream?index}] = Net_${stream?index}(data_mean=data_mean, data_std=data_std)
        self.networks[${stream?index}].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[${stream?index}].hybridize()
        self.networks[${stream?index}](<#list tc.getStreamInputDimensions(stream) as dimensions>mx.nd.zeros((${tc.join(dimensions, ",")},), ctx=context)<#sep>, </#list>)
</#if>
</#list>

<#list tc.architecture.unrolls as unroll>
<#list unroll.getBodiesForAllTimesteps() as body>
<#if body.isTrainable()>
        self.networks[${tc.architecture.streams?size + body?index}] = Net_${tc.architecture.streams?size + body?index}(data_mean=data_mean, data_std=data_std)
        self.networks[${tc.architecture.streams?size + body?index}].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[${tc.architecture.streams?size + body?index}].hybridize()
        self.networks[${tc.architecture.streams?size + body?index}](<#list tc.getStreamInputDimensions(body) as dimensions>mx.nd.zeros((${tc.join(dimensions, ",")},), ctx=context)<#sep>, </#list>)
</#if>
</#list>
</#list>

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        for i, network in self.networks.items():
            network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)
