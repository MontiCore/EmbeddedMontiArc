<#-- (c) https://github.com/MontiCore/monticore -->
<#assign axis = (element.axis + 1)?c>

<#if mode == "FORWARD_FUNCTION">
        ${element.name} = torch.cat((${tc.join(element.inputs, ", ")}), dim=${axis})
<#include "OutputShape.ftl">
</#if>