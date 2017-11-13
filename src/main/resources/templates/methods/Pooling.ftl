<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.Pooling(data=${tc.name(previous)}<#if false>,
        global_pool=None</#if><#if false>,
        cudnn_off=None</#if>,
        kernel=${tc.print(method,"kernel")},
        pool_type="${tc.print(method,"pool_type")}"<#if (method.get("pooling_convention"))??>,
        pooling_convention="${tc.print(method,"pooling_convention")}"</#if><#if (method.get("stride"))??>,
        stride=${tc.print(method,"stride")}</#if><#if (method.get("pad"))??>,
        pad=${tc.print(method,"pad")}</#if>,
        name="${tc.name(layer)}"<#if false>,
        attr=None</#if><#if false>,
        out=None</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("Pooling")
        .SetParam("kernel", Shape${tc.print(method,"kernel")})
        .SetParam("pool_type", "${tc.print(method,"pool_type")}") /*avg,max,sum */<#if (method.get("pooling_convention"))??>
        .SetParam("pooling_convention", "${tc.print(method,"pooling_convention")}")</#if><#if false>
        .SetParam("global_pool", false)</#if><#if (method.get("stride"))??>
        .SetParam("stride", Shape${tc.print(method,"stride")})</#if><#if (method.get("pad"))??>
        .SetParam("pad", Shape${tc.print(method,"pad")})</#if>
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>