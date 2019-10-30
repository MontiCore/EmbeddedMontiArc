<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${element.inputs[element.index]}
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = ${element.inputs[element.index]}
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name} = ${element.inputs[element.index]};
</#if>
