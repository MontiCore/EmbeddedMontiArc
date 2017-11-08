<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.LRN(data=${tc.name(previous)}<#if (method.get("alpha"))??>,
        alpha=${tc.print(method,"alpha")}</#if><#if (method.get("beta"))??>,
        beta=${tc.print(method,"beta")}</#if><#if (method.get("knorm"))??>,
        knorm=${tc.print(method,"knorm")}</#if>,
        nsize=${tc.print(method,"nsize")},
        name="${tc.name(layer)}"<#if false>,
        attr=</#if><#if false>,
        out=</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("LRN")
        .SetParam("nsize", ${tc.print(method,"nsize")})<#if (method.get("alpha"))??>
        .SetParam("alpha", ${tc.print(method,"alpha")})</#if><#if (method.get("beta"))??>
        .SetParam("beta", ${tc.print(method,"beta")})</#if><#if (method.get("knorm"))??>
        .SetParam("knorm", ${tc.print(method,"knorm")})</#if>
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>