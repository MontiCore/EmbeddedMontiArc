<#if target == "PYTHON">
${tc.name(layer)} = mx.symbol.SoftmaxOutput(data=${tc.name(previous)}<#if false>,
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
<#elseif target == "CPLUSPLUS">
auto ${tc.name(layer)} = Operator("SoftmaxOutput")<#if (method.get("grad_scale"))??>
        .SetParam("grad_scale", ${tc.print(method,"grad_scale")})</#if><#if false>
        .SetParam("ignore_label", -1)</#if><#if false>
        .SetParam("multi_output", false)</#if><#if false>
        .SetParam("use_ignore", false)</#if><#if false>
        .SetParam("normalization", "null") /*batch,null,valid */</#if>
        .SetInput("data", ${tc.name(previous)})<#if false>
        .SetInput("label", target_label)</#if>
        .CreateSymbol("softmax");
</#if>