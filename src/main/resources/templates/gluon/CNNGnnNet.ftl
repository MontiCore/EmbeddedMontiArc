<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import mxnet as mx
import numpy as np
import math
import os
import abc
import warnings
import sys
from mxnet import gluon, nd
<#if tc.containsAdaNet()>
from mxnet.gluon import nn, HybridBlock, Block
from numpy import log, product,prod,sqrt
from mxnet.ndarray import zeros,zeros_like
sys.path.insert(1, '${tc.architecture.getAdaNetUtils()}')
from AdaNetConfig import AdaNetConfig
import CoreAdaNet
</#if>
<#if tc.architecture.useDgl>
# Add dgl layers here
from dgl.nn.mxnet import GraphConv
</#if>
<#if tc.architecture.customPyFilesPath??>
sys.path.insert(1, '${tc.architecture.customPyFilesPath}')
from custom_layers import *
</#if>

class ZScoreNormalization(gluon.HybridBlock):
    def __init__(self, data_mean, data_std, **kwargs):
        super(ZScoreNormalization, self).__init__(**kwargs)
        with self.name_scope():
            self.data_mean = self.params.get('data_mean', shape=data_mean.shape,
                init=mx.init.Constant(data_mean.asnumpy().tolist()), differentiable=False)
            self.data_std = self.params.get('data_std', shape=data_mean.shape,
                init=mx.init.Constant(data_std.asnumpy().tolist()), differentiable=False)

    def hybrid_forward(self, F, x, data_mean, data_std):
        x = F.broadcast_sub(x, data_mean)
        x = F.broadcast_div(x, data_std)
        return x


class Padding(gluon.HybridBlock):
    def __init__(self, padding, **kwargs):
        super(Padding, self).__init__(**kwargs)
        with self.name_scope():
            self.pad_width = padding

    def hybrid_forward(self, F, x):
        x = F.pad(data=x,
            mode='constant',
            pad_width=self.pad_width,
            constant_value=0)
        return x

<#if tc.architecture.useDgl>
class NoNormalization(gluon.Block):
    def __init__(self, **kwargs):
        super(NoNormalization, self).__init__(**kwargs)
    def forward(self, x):
        return x
<#else>
class NoNormalization(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(NoNormalization, self).__init__(**kwargs)
    def hybrid_forward(self, F, x):
        return x
</#if>

<#if tc.architecture.useDgl>
class Reshape(gluon.Block):
    def __init__(self, shape, **kwargs):
        super(Reshape, self).__init__(**kwargs)
        with self.name_scope():
            self.shape = shape

    def forward(self, x):
        return nd.reshape(data=x, shape=self.shape)
<#else>
class Reshape(gluon.HybridBlock):
    def __init__(self, shape, **kwargs):
        super(Reshape, self).__init__(**kwargs)
        with self.name_scope():
            self.shape = shape

    def hybrid_forward(self, F, x):
        return F.reshape(data=x, shape=self.shape)
</#if>

class CustomRNN(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomRNN, self).__init__(**kwargs)
        with self.name_scope():
            self.rnn = gluon.rnn.RNN(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                     bidirectional=bidirectional, activation='tanh', layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.rnn(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)


class CustomLSTM(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomLSTM, self).__init__(**kwargs)
        with self.name_scope():
            self.lstm = gluon.rnn.LSTM(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                       bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0, state1):
        output, [state0, state1] = self.lstm(data, [F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)])
        return output, F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)


class CustomGRU(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomGRU, self).__init__(**kwargs)
        with self.name_scope():
            self.gru = gluon.rnn.GRU(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                     bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.gru(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)

<#if tc.containsAdaNet()>
# Blocks needed for AdaNet are generated below
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.containsAdaNet()>
${tc.include(networkInstruction.body, "ADANET_CONSTRUCTION")}
<#assign outblock = networkInstruction.body.getElements()[1].getDeclaration().getBlock("outBlock")>
<#assign block = networkInstruction.body.getElements()[1].getDeclaration().getBlock("block")>
<#assign inblock = networkInstruction.body.getElements()[1].getDeclaration().getBlock("inBlock")>
class Net_${networkInstruction?index}(gluon.HybridBlock):
    # this is a dummy network during the AdaNet generation it gets overridden
    # it is only here so many if tags in the .ftl files can be avoided
    def __init__(self,**kwargs):
        super(Net_${networkInstruction?index},self).__init__(**kwargs)
        with self.name_scope():
            self.AdaNet = True
            self.dummy = nn.Dense(units=1)

    def hybrid_forward(self,F,x):
        return self.dummy(x)
FullyConnected = AdaNetConfig.DEFAULT_BLOCK.value
DataClass_${networkInstruction?index} = CoreAdaNet.DataClass(
    <#if outblock.isPresent()>
        outBlock = ${outblock.get().name},
    <#else>
        outBlock = None,
    </#if>
    <#if inblock.isPresent()>
        inBlock = ${inblock.get().name},
    <#else>
        inBlock = None,
    </#if>
    <#if block.isPresent()>
        block = ${block.get().name},
    <#else>
        block = None,
    </#if>
        model_shape = ${tc.getDefinedOutputDimension()})
</#if>
</#list>
<#else>
<#list tc.architecture.networkInstructions as networkInstruction>
#Stream ${networkInstruction?index}
<#list networkInstruction.body.episodicSubNetworks as elements>
class EpisodicSubNet_${elements?index}(gluon.HybridBlock):
<#if elements?index == 0 >
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(EpisodicSubNet_${elements?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, elements?index, "ARCHITECTURE_DEFINITION")}
    
            pass
    
    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
${tc.include(networkInstruction.body, elements?index, "FORWARD_FUNCTION")}
        return [[${tc.join(tc.getSubnetOutputNames(elements), ", ")}]]
<#else>
    def __init__(self, mx_context=None, **kwargs):
        super(EpisodicSubNet_${elements?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, elements?index, "ARCHITECTURE_DEFINITION")}
    
            pass
    
    def hybrid_forward(self, F, *args):
${tc.include(networkInstruction.body, elements?index, "FORWARD_FUNCTION")}
        retNames = [${tc.join(tc.getSubnetOutputNames(elements), ", ")}]
        ret = []
        for elem in retNames:
            if isinstance(elem, list) and len(elem) >= 2:
                for elem2 in elem: 
                    ret.append(elem2)
            else:
                ret.append(elem)
        return [ret, [${tc.getSubnetInputNames(elements)[0]}full_, ind_${tc.join(tc.getSubnetInputNames(elements), ", ")}]]
</#if>

</#list>
# Using Block when using DGL library, otherwise use HybridBlock
<#if tc.architecture.useDgl>
class Net_${networkInstruction?index}(gluon.Block):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        with self.name_scope():
<#if networkInstruction.body.episodicSubNetworks?has_content>
<#list networkInstruction.body.episodicSubNetworks as elements>
<#if elements?index == 0>
            self.episodicsubnet0_ = EpisodicSubNet_${elements?index}(data_mean, data_std, mx_context)

            self.episodic_sub_nets = []

<#else>
            self.episodic_sub_nets.append(EpisodicSubNet_${elements?index}(mx_context=mx_context))
            self.register_child(self.episodic_sub_nets[${elements?index - 1}])

</#if>
</#list>
<#else>
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}
</#if>
            pass

    def forward(self, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
<#if networkInstruction.body.episodicSubNetworks?has_content>
<#list networkInstruction.body.episodicSubNetworks as elements>
<#if elements?index == 0>
        episodicsubnet${elements?index}_ = self.episodicsubnet${elements?index}_(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")})  
<#else>
        episodicsubnet${elements?index}_ = self.episodic_sub_nets[${elements?index-1}](*episodicsubnet${elements?index - 1}_[0])
</#if>
</#list>
        return [episodicsubnet${networkInstruction.body.episodicSubNetworks?size - 1}_[0], [<#list networkInstruction.body.episodicSubNetworks as elements><#if elements?index != 0>episodicsubnet${elements?index}_[1], </#if></#list>]]
<#else>
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}], [attention_output_]]
<#else>
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}]]
</#if>
</#if>
<#else>
class Net_${networkInstruction?index}(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        with self.name_scope():
        <#if networkInstruction.body.episodicSubNetworks?has_content>
            <#list networkInstruction.body.episodicSubNetworks as elements>
<#if elements?index == 0>
            self.episodicsubnet0_ = EpisodicSubNet_${elements?index}(data_mean, data_std, mx_context)

            self.episodic_sub_nets = []

<#else>
            self.episodic_sub_nets.append(EpisodicSubNet_${elements?index}(mx_context=mx_context))
            self.register_child(self.episodic_sub_nets[${elements?index - 1}])

</#if>
</#list>
<#else>
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}
</#if>
            pass

    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
<#if networkInstruction.body.episodicSubNetworks?has_content>
<#list networkInstruction.body.episodicSubNetworks as elements>
<#if elements?index == 0>
        episodicsubnet${elements?index}_ = self.episodicsubnet${elements?index}_(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")})
<#else>
        episodicsubnet${elements?index}_ = self.episodic_sub_nets[${elements?index-1}](*episodicsubnet${elements?index - 1}_[0])
</#if>
</#list>
        return [episodicsubnet${networkInstruction.body.episodicSubNetworks?size - 1}_[0], [<#list networkInstruction.body.episodicSubNetworks as elements><#if elements?index != 0>episodicsubnet${elements?index}_[1], </#if></#list>]]
<#else>
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}], [attention_output_]]
<#else>
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}]]
</#if>
</#if>
</#if>
</#list>
</#if>
