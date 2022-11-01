<#-- (c) https://github.com/MontiCore/monticore -->
# (c) https://github.com/MontiCore/monticore
import inspect
import json
import logging
import os
import pathlib
import shutil
import sys
import typing as t
import warnings

import mxnet as mx

<#if tc.architecture.customPyFilesPath??>
sys.path.insert(1, '${tc.architecture.customPyFilesPath}')
from custom_layers import *
</#if>

<#list tc.architecture.networkInstructions as networkInstruction>
<#if tc.containsAdaNet()>
from CNNNet_${tc.fullArchitectureName} import Net_${networkInstruction?index},DataClass_${networkInstruction?index}
<#else>
from CNNNet_${tc.fullArchitectureName} import Net_${networkInstruction?index}
</#if>
</#list>

from CNNDatasets_${tc.fullArchitectureName} import Dataset, TrainingDataset

log = logging.getLogger(__name__)

class ${tc.fileNameWithoutEnding}: # pylint: disable=invalid-name
    _model_basedir_ = pathlib.Path("model", "${tc.componentName}")

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        self.networks = {}
        self.dataset: TrainingDataset = None
        <#if tc.containsAdaNet()>
        self.dataClass = {}
        </#if>
<#if (tc.weightsPath)??>
        self._weights_dir_ = "${tc.weightsPath}/"
<#else>
        self._weights_dir_ = None
</#if>

    def get_model_dir(self, epoch: int, dataset: Dataset = None) -> pathlib.Path:
        if not dataset and not self.dataset:
            return self._model_basedir_ / "model" / str(epoch)
        elif not dataset:
            return self._model_basedir_ / "model" / self.dataset.id / str(epoch)
        else:
            return self._model_basedir_ / "model" / dataset.id / str(epoch)
        fi

    def load(self, context): # pylint: disable=unused-argument
        earliestLastEpoch = None

        for i, network in self.networks.items():
            lastEpoch = 0
            param_file = None
            if hasattr(network, 'episodic_sub_nets'):
                num_episodic_sub_nets = len(network.episodic_sub_nets)
                lastMemEpoch = [0]*num_episodic_sub_nets
                mem_files = [None]*num_episodic_sub_nets

            if self.dataset:
                prefix = self.get_model_dir(i)

                models_to_remove: t.Set[pathlib.Path] = {
                    prefix / "newest-0000.params",
                    prefix / "newest-symbol.json"
                }

                if hasattr(network, 'episodic_sub_nets'):
                    for j in range(len(network.episodic_sub_nets) + 1):
                        models_to_remove.add(prefix / "newest_episodic_sub_net" + str(j) + "-0000.params")
                        models_to_remove.add(prefix / "newest_episodic_sub_net" + str(j) + "-symbol.json")
                        models_to_remove.add(prefix / "newest_loss" + str(j) + "-0000.params")
                        models_to_remove.add(prefix / "newest_loss" + str(j) + "-symbol.json")

                        if j > 0:
                            models_to_remove.add(prefix / "newest_episodic_query_net_" + str(j) + "-0000.params")
                            models_to_remove.add(prefix / "newest_episodic_query_net_" + str(j) + "-symbol.json")
                            models_to_remove.add(prefix / "newest_episodic_memory_sub_net_" + str(j) + "-symbol.json")

                for trained_model in models_to_remove:
                    if trained_model.exists():
                        try:
                            trained_model.unlink()
                        except OSError:
                            log.info("Error during deletion of file %s", trained_model.absolute())

                for file in prefix.glob("*"):
                    if file.suffix == ".params" and not "loss" in file.name:
                        epoch = int(file.stem)
                        if epoch >= lastEpoch:
                            lastEpoch = epoch
                            param_file = file
                    elif hasattr(network, 'episodic_sub_nets') and file.stem.startswith("episodic_memory_sub_net_"):
                        relMemPathInfo = file.name.replace("episodic_memory_sub_net_", "").split("-")
                        memSubNet = int(relMemPathInfo[0])
                        memEpochStr = relMemPathInfo[1]
                        memEpoch = int(memEpochStr)
                        if memEpoch >= lastMemEpoch[memSubNet-1]:
                            lastMemEpoch[memSubNet-1] = memEpoch
                            mem_files[memSubNet-1] = file

                if param_file is None:
                    earliestLastEpoch = 0
                else:
                    logging.info("Loading checkpoint: %s", param_file)
                    network.load_parameters(str(param_file))
                    if hasattr(network, 'episodic_sub_nets'):
                        for j, sub_net in enumerate(network.episodic_sub_nets):
                            if mem_files[j] != None:
                                logging.info("Loading Replay Memory: " + mem_files[j])
                                mem_layer = [param for param in inspect.getmembers(sub_net, lambda x: not(inspect.isroutine(x))) if param[0].startswith("memory")][0][1]
                                mem_layer.load_memory(self.get_model_dir(i) + mem_files[j])

                    if earliestLastEpoch == None or lastEpoch + 1 < earliestLastEpoch:
                        earliestLastEpoch = lastEpoch + 1

        return earliestLastEpoch

    def load_pretrained_weights(self, context, dataset):
        for i, network in self.networks.items():
            prefix = self.get_model_dir(i, dataset)
            weights_dir = pathlib.Path(self._weights_dir_ or prefix)
            param_file = None
            if hasattr(network, 'episodic_sub_nets'):
                num_episodic_sub_nets = len(network.episodic_sub_nets)
                lastMemEpoch = [0] * num_episodic_sub_nets
                mem_files = [None] * num_episodic_sub_nets

            if weights_dir.is_dir():
                lastEpoch = 0
                for file in weights_dir.glob("*"):
                    if file.suffix == ".params" and file.stem.isdigit():
                        epoch = int(file.stem)
                        if epoch >= lastEpoch:
                            lastEpoch = epoch
                            param_file = file
                    elif hasattr(network, 'episodic_sub_nets') and file.stem.startswith("episodic_memory_sub_net_"):
                        relMemPathInfo = file.name.replace("episodic_memory_sub_net_", "").split("-")
                        memSubNet = int(relMemPathInfo[0])
                        memEpochStr = relMemPathInfo[1]
                        memEpoch = int(memEpochStr)
                        if memEpoch >= lastMemEpoch[memSubNet-1]:
                            lastMemEpoch[memSubNet-1] = memEpoch
                            mem_files[memSubNet-1] = file

                logging.info("Loading pretrained weights: %s", str(param_file))
                network.load_parameters(str(param_file), allow_missing=True, ignore_extra=True)
                if hasattr(network, 'episodic_sub_nets'):
                    assert lastEpoch == lastMemEpoch
                    for j, sub_net in enumerate(network.episodic_sub_nets):
                        if mem_files[j] != None:
                            logging.info("Loading pretrained Replay Memory: %s", mem_files[j])
                            mem_layer = \
                            [param for param in inspect.getmembers(sub_net, lambda x: not (inspect.isroutine(x))) if
                                param[0].startswith("memory")][0][1]
                            mem_layer.load_memory(self.get_model_dir(i) + mem_files[j])
            else:
                logging.info("No pretrained weights available at: %s. Will not use pretrained weights.", str(weights_dir))

    def construct(self, context, batch_size=1, data_mean=None, data_std=None):
<#list tc.architecture.networkInstructions as networkInstruction>
        <#if tc.containsAdaNet()>
        self.networks[${networkInstruction?index}] = Net_${networkInstruction?index}()
        self.dataClass[${networkInstruction?index}] = DataClass_${networkInstruction?index}
        <#else>
        self.networks[${networkInstruction?index}] = Net_${networkInstruction?index}(batch_size=batch_size, data_mean=data_mean, data_std=data_std, mx_context=context, prefix="")
        </#if>
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.networks[${networkInstruction?index}].collect_params().initialize(self.weight_initializer, force_reinit=False, ctx=context)
        self.networks[${networkInstruction?index}].hybridize()
<#if !(tc.architecture.useDgl)>
        self.networks[${networkInstruction?index}](<#list tc.getStreamInputDimensions(networkInstruction.body) as dimensions><#if tc.cutDimensions(dimensions)[tc.cutDimensions(dimensions)?size-1] == "1" && tc.cutDimensions(dimensions)?size != 1>mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0])<#else>mx.nd.zeros((batch_size, ${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0])</#if><#sep>, </#list>)
</#if>
<#if networkInstruction.body.episodicSubNetworks?has_content>
        self.networks[0].episodicsubnet0_(<#list tc.getStreamInputDimensions(networkInstruction.body) as dimensions><#if tc.cutDimensions(dimensions)[tc.cutDimensions(dimensions)?size-1] == "1" && tc.cutDimensions(dimensions)?size != 1>mx.nd.zeros((${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0])<#else>mx.nd.zeros((batch_size, ${tc.join(tc.cutDimensions(dimensions), ",")},), ctx=context[0])</#if><#sep>, </#list>)
</#if>
</#list>

        if not os.path.exists(self._model_basedir_):
            os.makedirs(self._model_basedir_)
<#if !(tc.architecture.useDgl)>
        for i, network in self.networks.items():
            path = self.get_model_dir(i)
            path.mkdir(parents=True, exist_ok=True)
            network.export(path, epoch=0)
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