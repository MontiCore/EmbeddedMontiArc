<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.MAERegressionOutput(data=${tc.name(previous)}<#if false>,
        label=None</#if><#if (method.get("grad_scale"))??>,
        grad_scale=${tc.print(method,"grad_scale")}</#if><#if false>,
        name=</#if><#if false>,
        attr=</#if><#if false>,
        out=</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("MAERegressionOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("mae");
</#if>