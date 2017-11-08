<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.Convolution(data=${tc.name(previous)}<#if false>,
        weight=None</#if><#if false>,
        bias=None</#if>,
        kernel=${tc.print(method, "kernel")}<#if (method.get("stride"))??>,
        stride=${tc.print(method, "stride")}</#if><#if false>,
        dilate=None</#if><#if (method.get("pad"))??>,
        pad=${tc.print(method, "pad")}</#if>,
        num_filter=${tc.print(method, "num_filter")}<#if false>,
        num_group=None</#if><#if false>,
        workspace=None</#if><#if (method.get("no_bias"))??>,
        no_bias=${tc.print(method, "no_bias")?capitalize}</#if><#if false>,
        cudnn_tune=None</#if><#if false>,
        cudnn_off=None</#if><#if false>,
        layout=None</#if>,
        name="${tc.name(layer)}"<#if false>,
        attr=None</#if>)
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("Convolution")
        .SetParam("kernel", Shape${tc.print(method, "kernel")})
        .SetParam("num_filter", ${tc.print(method, "num_filter")})<#if (method.get("stride"))??>
        .SetParam("stride", Shape${tc.print(method, "stride")})</#if><#if false>
        .SetParam("dilate", Shape(1, 1))</#if><#if (method.get("pad"))??>
        .SetParam("pad", Shape${tc.print(method, "pad")})</#if><#if false>
        .SetParam("num_group", 1)</#if><#if false>
        .SetParam("workspace", 512)</#if><#if (method.get("no_bias"))??>
        .SetParam("no_bias", false)</#if>
        .SetInput("data", ${tc.name(previous)})
        .CreateSymbol("${tc.name(layer)}");
</#if>