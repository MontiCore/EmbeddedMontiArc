import mxnet as mx
import logging
import os

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.isTrainable()>
from CNNNet_${tc.fullArchitectureName} import Net_${networkInstruction?index}
</#if>
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
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.isTrainable()>
        self.networks[${networkInstruction?index}] = Net_${networkInstruction?index}(data_mean=data_mean, data_std=data_std)
        self.networks[${networkInstruction?index}].collect_params().initialize(self.weight_initializer, ctx=context)
        self.networks[${networkInstruction?index}].hybridize()
        self.networks[${networkInstruction?index}](<#list tc.getStreamInputDimensions(networkInstruction.body) as dimensions>mx.nd.zeros((${tc.join(dimensions, ",")},), ctx=context)<#sep>, </#list>)
</#if>
</#list>

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        for i, network in self.networks.items():
            network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)
