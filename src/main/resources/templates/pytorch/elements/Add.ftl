<#-- (c) https://github.com/MontiCore/monticore -->

<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${tc.join(element.inputs, " + ")}
<#include "OutputShape.ftl">
</#if>