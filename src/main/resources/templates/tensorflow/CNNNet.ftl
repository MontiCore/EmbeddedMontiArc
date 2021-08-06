<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import logging
import os
import errno
import shutil
import h5py
import sys
import numpy as np

import tensorflow as tf

    _input_names_ = [<#list tc.architectureInputs as inputName>'${inputName?keep_before_last("_")}'<#sep>, </#list>]
    _output_names_ = [${tc.join(tc.architectureOutputs, ",", "'", "label'")}]
    _output_shapes_ = [<#list tc.architecture.outputs as output>(${tc.join(output.ioDeclaration.type.dimensions, ",")},)</#list>]
    _weight_constraint_ = None
    _regularizer_ = None

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
class Net_${stream?index}:
    def __init__(self, data_mean=None, data_std=None):
        self.data_mean = data_mean
        self.data_std = data_std
        self.model = None

    def forward(self):
${tc.include(stream)}
        
        self.model = tf.keras.models.Model(inputs=input_tensors, outputs=[${tc.join(tc.getStreamOutputNames(stream), ", ")}])
                       
        return ${tc.join(tc.getStreamOutputNames(stream), ", ")}

</#if>
</#list>

                       
    def construct(self, data_mean=None, data_std=None):
	
        input_tensors = []
        output_names = []
	
${tc.include(tc.architecture.streams[0])}
            
        
        self.model = tf.keras.models.Model(inputs=input_tensors, outputs=[${tc.join(tc.architectureOutputs, ",")}])
             
				
        out_nodes = [node for node in self.module.outputs] 
        tf.identity(out_nodes[0], name="output_0")
