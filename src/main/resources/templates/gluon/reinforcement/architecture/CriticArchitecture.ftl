architecture ${architectureName}() {
    def input ${stateType}<#if stateRange??>(<#if stateRange.isLowerLimitInfinity()>-oo<#else>${stateRange.lowerLimit.get()}</#if>:<#if stateRange.isUpperLimitInfinity()>oo<#else>${stateRange.upperLimit.get()}</#if>)</#if>^{<#list stateDimension as d>${d}<#if d?has_next>,</#if></#list>} state
    def input ${actionType}<#if actionRange??>(<#if actionRange.isLowerLimitInfinity()>-oo<#else>${actionRange.lowerLimit.get()}</#if>:<#if actionRange.isUpperLimitInfinity()>oo<#else>${actionRange.upperLimit.get()}</#if>)</#if>^{<#list actionDimension as d>${d}<#if d?has_next>,</#if></#list>} action
    def output Q(-oo:oo)^{1} qvalue

    ${implementation}->FullyConnected(units=1)->qvalue;
}