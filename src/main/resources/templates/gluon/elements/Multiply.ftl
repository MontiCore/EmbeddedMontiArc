<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${tc.join(element.inputs, " * ")}
<#elseif mode == "PYTHON_INLINE">
                    ${element.name} = ${tc.join(element.inputs, " * ")}
<#elseif mode == "CPP_INLINE">
    vector<float> ${element.name}(${element.inputs[0]}.size());
    for (size_t i = 0; i != ${element.name}.size(); ++i) {
        ${element.name}[i] = ${tc.join(element.inputs, " * ", "", "[i]")};
    }
</#if>