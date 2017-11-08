<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.FullyConnected(data=${tc.name(previous)}<#if false>,
        weight=None</#if><#if false>,
        bias=None</#if>,
        num_hidden=${tc.print(method, "num_hidden")}<#if (method.get("no_bias"))??>,
        no_bias=${tc.print(method,"no_bias")?capitalize}</#if><#if false>,
        flatten=None</#if>,
        name="${tc.name(layer)}"<#if false>,
        attr=None</#if><#if false>,
        out=None</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("FullyConnected")
        .SetParam("num_hidden", ${tc.print(method, "num_hidden")})<#if (method.get("no_bias"))??>
        .SetParam("no_bias", ${tc.print(method,"no_bias")})</#if>
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>