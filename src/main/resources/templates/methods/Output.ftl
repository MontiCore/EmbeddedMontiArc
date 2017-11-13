<#if target == "PYTHON">
outputFc = mx.symbol.FullyConnected(data=${tc.name(previous)}, num_hidden=${outputPort.dimension})
<#if tc.print(method,"type")=="softmax">
    ${tc.name(layer)} = mx.symbol.SoftmaxOutput(data=outputFc<#if false>,
        label=None</#if><#if (method.get("grad_scale"))??>,
        grad_scale=${tc.print(method,"grad_scale")}</#if><#if false>,
        ignore_label=None</#if><#if false>,
        multi_output=None</#if><#if false>,
        use_ignore=None</#if><#if false>,
        preserve_shape=None</#if><#if false>,
        normalization=None</#if><#if false>,
        out_grad=None</#if>,
        name="softmax"<#if false>,
        attr=None</#if><#if false>,
        out=None</#if>)
<#elseif tc.print(method,"type")=="linear">
    ${tc.name(layer)} = mx.symbol.LinearRegressionOutput(data=${tc.name(previous)}<#if false>,
        label=None</#if><#if (method.get("grad_scale"))??>,
        grad_scale=${tc.print(method,"grad_scale")}</#if><#if false>,
        name=</#if><#if false>,
        attr=</#if><#if false>,
        out=</#if>)
<#elseif tc.print(method,"type")=="logistic">
    ${tc.name(layer)} = mx.symbol.LogisticRegressionOutput(data=${tc.name(previous)}<#if false>,
        label=None</#if><#if (method.get("grad_scale"))??>,
        grad_scale=${tc.print(method,"grad_scale")}</#if><#if false>,
        name="mae"</#if><#if false>,
        attr=</#if><#if false>,
        out=</#if>)
<#elseif tc.print(method,"type")=="mae">
    ${tc.name(layer)} = mx.symbol.MAERegressionOutput(data=${tc.name(previous)}<#if false>,
        label=None</#if><#if (method.get("grad_scale"))??>,
        grad_scale=${tc.print(method,"grad_scale")}</#if><#if false>,
        name=</#if><#if false>,
        attr=</#if><#if false>,
        out=</#if>)
</#if>

<#elseif target == "CPLUSPLUS">
outputFc = mx.symbol.FullyConnected(data=${tc.name(previous)}, num_hidden=${outputPort.dimension})
<#if tc.print(method,"type")=="softmax">
    auto ${tc.name(layer)} = Operator("SoftmaxOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if><#if false>
        .SetParam("ignore_label", -1)</#if><#if false>
        .SetParam("multi_output", false)</#if><#if false>
        .SetParam("use_ignore", false)</#if><#if false>
        .SetParam("normalization", "null") /*batch,null,valid */</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("softmax");
<#elseif tc.print(method,"type")=="linear">
    auto ${tc.name(layer)} = Operator("LinearRegressionOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("mae");
<#elseif tc.print(method,"type")=="logistic">
    auto ${tc.name(layer)} = Operator("LogisticRegressionOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("mae");
<#elseif tc.print(method,"type")=="mae">
    auto ${tc.name(layer)} = Operator("MAERegressionOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("mae");
</#if>
</#if>