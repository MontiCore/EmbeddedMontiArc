<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import mxnet as mx
import logging
import os
import shutil
import warnings
import inspect
import sys
<#if tc.architecture.customPyFilesPath??>
sys.path.insert(1, '${tc.architecture.customPyFilesPath}')
from custom_layers import *
</#if>

<#list tc.architecture.networkInstructions as networkInstruction>
from gnn.CNNGnnNet_${tc.fullArchitectureName} import Net_${networkInstruction?index}
</#list>

class CNNGnnCreator:
    _model_dir_ = "model/${tc.componentName}/"
    _model_prefix_ = "model"

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        self.networks = {}
<#if (tc.weightsPath)??>
        self._weights_dir_ = "${tc.weightsPath}/"
<#else>
        self._weights_dir_ = None
</#if>

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
                    if ".params" in file and self._model_prefix_ + "_" + str(i) in file and not "loss" in file:
                        epochStr = file.replace(".params", "").replace(self._model_prefix_ + "_" + str(i) + "-", "")
                        epoch = int(epochStr)
                        if epoch >= lastEpoch:
                            lastEpoch = epoch
                            param_file = file

            if param_file is None:
                earliestLastEpoch = 0
            else:
                logging.info("Loading checkpoint: " + param_file)
                network.load_parameters(self._model_dir_ + param_file)

                if earliestLastEpoch == None or lastEpoch + 1 < earliestLastEpoch:
                    earliestLastEpoch = lastEpoch + 1

        return earliestLastEpoch

    def load_pretrained_weights(self, context):
        if os.path.isdir(self._model_dir_):
            shutil.rmtree(self._model_dir_)
        if self._weights_dir_ is not None:
            for i, network in self.networks.items():
                # param_file = self._model_prefix_ + "_" + str(i) + "_newest-0000.params"
                param_file = None

                if os.path.isdir(self._weights_dir_):
                    lastEpoch = 0

                    for file in os.listdir(self._weights_dir_):

                        if ".params" in file and self._model_prefix_ + "_" + str(i) in file and not "loss" in file:
                            epochStr = file.replace(".params","").replace(self._model_prefix_ + "_" + str(i) + "-","")
                            epoch = int(epochStr)
                            if epoch >= lastEpoch:
                                lastEpoch = epoch
                                param_file = file

                    logging.info("Loading pretrained weights: " + self._weights_dir_ + param_file)
                    network.load_parameters(self._weights_dir_ + param_file, allow_missing=True, ignore_extra=True)
                else:
                    logging.info("No pretrained weights available at: " + self._weights_dir_ + param_file)

    def construct(self, context, data_mean=None, data_std=None, graph=None):
<#list tc.architecture.networkInstructions as networkInstruction>
        self.networks[${networkInstruction?index}] = Net_${networkInstruction?index}(data_mean=data_mean, data_std=data_std, mx_context=context, prefix="")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.networks[${networkInstruction?index}].collect_params().initialize(self.weight_initializer, force_reinit=False, ctx=context)
        self.networks[${networkInstruction?index}].hybridize()
<#if tc.architecture.useDgl>
        self.networks[${networkInstruction?index}](<#list tc.getStreamInputDimensions(networkInstruction.body)
                      as dimensions><#if dimensions?index==0>graph<#elseif tc.cutDimensions(dimensions)[tc.cutDimensions(dimensions)?size-1] == "1" &&
                      tc.cutDimensions(dimensions)?size != 1>mx.nd.squeeze(mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},),
                      ctx=context[0]))<#else>mx.nd.squeeze(mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0]))</#if><#sep>, </#list>)
<#else>
        self.networks[${networkInstruction?index}](<#list tc.getStreamInputDimensions(networkInstruction.body)
                      as dimensions><#if tc.cutDimensions(dimensions)[tc.cutDimensions(dimensions)?size-1] == "1" &&
                      tc.cutDimensions(dimensions)?size != 1>mx.nd.squeeze(mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},),
                      ctx=context[0]))<#else>mx.nd.squeeze(mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0]))</#if><#sep>, </#list>)
</#if>
</#list>

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

<#if !(tc.architecture.useDgl)>
        for i, network in self.networks.items():
            network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)
</#if>
    def setWeightInitializer(self, initializer):
        self.weight_initializer = initializer

    def getInputs(self):
        inputs = {}
<#list tc.architecture.streams as stream>
<#assign dimensions = (tc.getStreamInputs(stream, false))>
<#assign domains = (tc.getStreamInputDomains(stream))>
<#list tc.getStreamInputVariableNames(stream, false) as name>
        input_dimensions = (${tc.join(dimensions[name], ",")},)
        input_domains = (${tc.join(domains[name], ",")},)
        inputs["${name}"] = input_domains + (input_dimensions,)
</#list>
</#list>
        return inputs

    def getOutputs(self):
        outputs = {}
<#list tc.architecture.streams as stream>
<#assign dimensions = (tc.getStreamOutputs(stream, false))>
<#assign domains = (tc.getStreamOutputDomains(stream))>
<#list tc.getStreamOutputVariableNames(stream, false) as name>
        output_dimensions = (${tc.join(dimensions[name], ",")},)
        output_domains = (${tc.join(domains[name], ",")},)
        outputs["${name}"] = output_domains + (output_dimensions,)
</#list>
</#list>
        return outputs

    def validate_parameters(self):
<#list tc.architecture.networkInstructions as networkInstruction>
${tc.include(networkInstruction.body, "PARAMETER_VALIDATION")}
</#list>        pass