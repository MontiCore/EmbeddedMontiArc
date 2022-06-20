<#-- (c) https://github.com/MontiCore/monticore -->
import torch
import torch.nn as nn
import torch.nn.functional as F
from dgl.nn import GraphConv

class ${tc.fileNameWithoutEnding}(nn.Module):
    def __init__(self):
        super(${tc.fileNameWithoutEnding}, self).__init__()
<#list tc.architecture.networkInstructions as networkInstruction>
${tc.include(networkInstruction.body,  "ARCHITECTURE_DEFINITION")}
</#list>
        pass

    def forward(self <#list tc.architectureInputs as inputName><#if inputName?index == tc.architectureInputs?seq_index_of(inputName)>, ${inputName} </#if></#list>):
<#list tc.architecture.networkInstructions as networkInstruction>
${tc.include(networkInstruction.body,  "FORWARD_FUNCTION")}

</#list>
