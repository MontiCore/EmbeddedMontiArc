${tc.name(layer)} = mx.symbol.SVMOutput(data=${tc.name(previous)}<#if false>,
        label=None</#if><#if (method.get("margin"))??>,
        margin=${tc.print(method,"margin")}</#if><#if (method.get("regularization_coefficient"))??>,
        regularization_coefficient=${tc.print(method,"regularization_coefficient")}</#if><#if (method.get("use_linear"))??>,
        use_linear=${tc.print(method,"use_linear")?capitalize}</#if><#if false>,
        name=</#if><#if false>,
        attr=</#if><#if false>,
        out=</#if>)