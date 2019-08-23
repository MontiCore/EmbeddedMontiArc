<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#-- TODO: Find solution for the CNNArch fix_gamma parameter of BatchNorm. Gluon does not provide this parameter-->
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = gluon.nn.BatchNorm()
            <#include "OutputShape.ftl">
<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})
</#if>
