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
<#if tc.architecture.useDgl>
# Add dgl layers here
from dgl.nn.mxnet import GraphConv, GATConv
</#if>
<#if tc.architecture.customPyFilesPath??>
sys.path.insert(1, '${tc.architecture.customPyFilesPath}')
from custom_layers import *
</#if>

# Needs to be adjusted to support DGL
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

# Needs to be adjusted to support DGL
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

<#list tc.architecture.networkInstructions as networkInstruction>
#Stream ${networkInstruction?index}

# Using Block when using DGL library, otherwise use HybridBlock
<#if tc.architecture.useDgl>
class Net_${networkInstruction?index}(gluon.Block):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}
            pass

    def forward(self, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}], [attention_output_]]
<#else>
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}]]
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
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}], [attention_output_]]
<#else>
        return [[${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}]]
</#if>
</#if>
</#list>
