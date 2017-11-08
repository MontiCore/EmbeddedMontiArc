<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.Dropout(data=${tc.name(previous)}<#if (method.get("p"))??>,
        p=${tc.print(method,"p")}</#if><#if (method.get("mode"))??>,
        mode=${tc.print(method,"mode")}</#if>,
        name="${tc.name(layer)}"<#if false>,
        attr=</#if><#if false>,
        out=</#if>)
<#elseif target == "PYTHON">
auto ${tc.name(layer)} = Operator("Dropout")<#if (method.get("p"))??>
        .SetParam("p", ${tc.print(method,"p")})</#if><#if (method.get("mode"))??>
        .SetParam("mode", ${tc.print(method,"mode")})</#if>
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>