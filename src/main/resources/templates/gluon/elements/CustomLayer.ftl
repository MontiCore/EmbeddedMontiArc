<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = ${element.element.arguments[0].rhs.value?remove_beginning("Optional[")?remove_ending("]")}.${element.element.arguments[0].rhs.value?remove_beginning("Optional[")?remove_ending("]")}(
            <#list element.element.arguments[1].rhs.value?split(",") as parameter>
                ${parameter?remove_beginning("Optional[")?remove_ending("]")}<#sep>, </#sep>
            </#list>)

<#elseif mode == "FORWARD_FUNCTION">
        ${element.name} = self.${element.name}(${input})

<#elseif mode == "PARAMETER_VALIDATION">
        ${element.name}temp = ${element.element.arguments[0].rhs.value?remove_beginning("Optional[")?remove_ending("]")}.${element.element.arguments[0].rhs.value?remove_beginning("Optional[")?remove_ending("]")}()
        parameters_with_type = ${element.name}temp.get_parameters()

        <#list element.element.arguments[1].rhs.value?split(", ") as parameter>
        if '${parameter?remove_beginning("Optional[")?remove_ending("]")?keep_before("=")}' in parameters_with_type:
             if isinstance(${parameter?remove_beginning("Optional[")?remove_ending("]")?keep_after("=")},parameters_with_type['${parameter?remove_beginning("Optional[")?remove_ending("]")?keep_before("=")}']) == False:
                  raise TypeError('Wrong ' + str(type(${parameter?remove_beginning("Optional[")?remove_ending("]")?keep_after("=")})) + ' of parameter \'${parameter?remove_beginning("Optional[")?remove_ending("]")?keep_before("=")}\' given in the model')
        else:
             raise AttributeError('Parameter of Layer not added to get_parameters function')

        </#list>

</#if>