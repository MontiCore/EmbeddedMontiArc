import logging
import mxnet as mx
import h5py
import os
from mxnet import gluon
import CNNCreator_${tc.getFullArchitectureName()} as creator
<#if tc.architecture.useDgl>
import dgl
</#if>

def load_data(path):
    graph = None
    data = h5py.File(path, 'r')
<#if tc.architecture.useDgl>
    graph, _ = dgl.load_graphs(os.path.splitext(path)[0] + "_graph")
</#if>
    return data, graph


def predict(path_to_h5, start=None, end=None, batch_size=1, model_path="../../../../", context="cpu"):
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
<#if tc.architecture.useDgl>
    logger.setLevel(logging.DEBUG)
</#if>
    data, graph = load_data(path_to_h5)
    if context == "gpu":
        mx_context = [mx.gpu()]
    else:
        if context != "cpu":
            logging.warning("%s is not a valid context argument. Switching to cpu..." % (context))
        mx_context = [mx.cpu()]

    net_creator = creator.CNNCreator_${tc.getFullArchitectureName()}()
<#if (tc.weightsPath)??>
    net_creator._weights_dir_ = "${tc.weightsPath}/"
<#else>
    net_creator._weights_dir_ = model_path + net_creator._model_dir_
</#if>
    net_creator.construct(context=mx_context)
<#if tc.architecture.useDgl>
    net_creator.load_pretrained_weights(context=mx_context)
<#else>
    net_creator._model_dir_ = net_creator._weights_dir_
    _ = net_creator.load(context=mx_context)
</#if>

    data_dict = {}
<#list tc.architectureInputs as input_name>
<#if input_name?index == tc.architectureInputs?seq_index_of(input_name)>
    if '${input_name?keep_before_last("_")}' in data:
        data_dict['${input_name}'] = data['${input_name?keep_before_last("_")}'][start:end]
</#if>
</#list>
    output_names = [${tc.join(tc.architectureOutputs, ",", "'", "label'")}]
    if len(data_dict) == 0:
        label = {}
        for out in output_names:
            label[out] = data[out]
        data_dict = label
    iter = mx.io.NDArrayIter(data=data_dict, batch_size=batch_size)
    outputs = None
    batches = 0

    for _, batch in enumerate(iter):
    <#if tc.architecture.useDgl>
        train_data_index = 0
        graph_ = [dgl.batch(graph[batch_size*batches:min(len(graph), batch_size*(batches+1))])]
    </#if>
<#assign input_index = 0>
<#list tc.architectureInputs as input_name>
<#if input_name?index == tc.architectureInputs?seq_index_of(input_name)>
    <#if tc.architecture.useDgl>
        <#if input_name != 'graph_'>
        if '${input_name}' in graph_[0].ndata:
            ${input_name} = [graph_[0].ndata['${input_name}']]
        elif '${input_name}' in graph_[0].edata:
            ${input_name} = [graph_[0].edata['${input_name}']]
        else:
            ${input_name} = gluon.utils.split_and_load(batch.data[train_data_index], ctx_list=mx_context, even_split=False)
            train_data_index += 1
        </#if>
    <#else>
        ${input_name} = gluon.utils.split_and_load(batch.data[${input_index}], ctx_list=mx_context, even_split=False)
        <#assign input_index++>
    </#if>
</#if>
</#list>
    <#list tc.architecture.networkInstructions as networkInstruction>
        net_ret = net_creator.networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), "[0], ")}[0])
        <#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
        ${outputName} = [net_ret[0][${outputName?index}]]
        <#if tc.getNameWithoutIndex(outputName) == tc.outputName>
        if outputs is None:
            outputs = ${outputName}[0]
        else:
            outputs = mx.nd.concat(outputs, ${outputName}[0], dim=0)
        </#if>
        </#list>
    </#list>
        batches += 1
    return outputs
