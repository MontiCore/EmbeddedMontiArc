<#-- (c) https://github.com/MontiCore/monticore -->
import torch
import torch.nn as nn
import torch.nn.functional as F

class CNNDataLoader_${tc.fullArchitectureName}(nn.Module):
     def __init__(self):
        super(CNNDataLoader_${tc.fullArchitectureName} , self).__init__()
        <#list tc.architecture.streams as stream>
        ${tc.include(stream,  "ARCHITECTURE_DEFINITION")} </#list>
        pass

     def forward(self, data):
       <#list tc.architecture.streams as stream>
        ${tc.include(stream,  "FORWARD_FUNCTION")}  </#list>
