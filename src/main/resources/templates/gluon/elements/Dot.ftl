<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.batch_dot(${tc.join(element.inputs, ", ")})
</#if>
