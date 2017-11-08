<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.Flatten(data=${tc.name(previous)},
        name="${tc.name(layer)}"<#if false>,
        attr=None</#if><#if false>,
        out=None</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("Flatten")
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>