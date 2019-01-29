<#-- TODO: May put this in an extra HybridBlock -->
<#assign mode = definition_mode.toString()>
<#if mode == "FORWARD_FUNCTION">
        ${element.name} = ${tc.join(element.inputs, " + ")}
</#if>