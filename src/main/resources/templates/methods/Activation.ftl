<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.Activation(data=${tc.name(previous)},
        act_type="${tc.print(method, "act_type")}",
        name="${tc.name(layer)}"<#if false>,
        attr=None</#if><#if false>,
        out=None</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("Activation")
        .SetParam("act_type", "${tc.print(method, "act_type")}") /*relu,sigmoid,softrelu,tanh */
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>