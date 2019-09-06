<#if mode == "FORWARD_FUNCTION">
        ${element.name} = gluon.Constant('${element.name}', ${element.constValue})
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = mx.nd.full((batch_size, 1,), ${element.constValue}, ctx=mx_context)
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}{${element.constValue}};
</#if>