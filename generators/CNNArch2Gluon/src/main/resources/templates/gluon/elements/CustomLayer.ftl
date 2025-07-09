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


<#elseif mode == "PARAMETER_VALIDATION">
    <#if element.element.arguments?has_content>
        ${element.name}temp = ${element.element.name}.${element.element.name}()
        parameters_with_type = ${element.name}temp.get_parameters()

        <#list element.element.arguments as argument>
        if '${argument.name}' in parameters_with_type:
             if isinstance(${argument.rhs.value.get()},parameters_with_type['${argument.name}']) == False:
                  raise TypeError('Wrong ' + str(type(${argument.rhs.value.get()})) + ' of parameter \'${argument.name}\' given in the model')
        else:
             raise AttributeError('Parameter of Layer not added to get_parameters function')

        </#list>
    </#if>

</#if>
