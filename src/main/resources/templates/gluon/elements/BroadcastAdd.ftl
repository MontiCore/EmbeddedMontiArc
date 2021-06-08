<#-- (c) https://github.com/MontiCore/monticore -->
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = F.broadcast_add(${tc.join(element.inputs, ",")})
</#if>
