<#if mode == "FORWARD_FUNCTION">
        ${element.name} = gluon.Const('${element.name}', ${element.constValue})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = nd.array(${element.constValue})
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}{${element.constValue}};
</#if>