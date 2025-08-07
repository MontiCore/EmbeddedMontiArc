<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">

            self.${element.name} = ${element.element.name}.${element.element.name}(<#rt>
                  <#list element.element.arguments as argument>
                        ${argument.name}=${argument.rhs.value.get()}<#sep>, </#sep><#t>
                  </#list>)<#lt>
            <#include "OutputShape.ftl">

<#elseif mode == "FORWARD_FUNCTION">

    <#if (element.inputs[1])??>
        ${element.name} = self.${element.name}(${tc.join(element.inputs, ", ")})
    <#else>
        ${element.name} = self.${element.name}(${input})
    </#if>
</#if>