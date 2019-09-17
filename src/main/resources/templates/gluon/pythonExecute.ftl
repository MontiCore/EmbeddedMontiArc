<#list tc.getLayerVariableMembers("batch_size")?keys as member>
<#if member?ends_with("_state_")>
                    encoder_state_ = self._networks[${tc.getLayerVariableMembers("batch_size")[member][1][0]}].${member?replace("_state_","_output_")}.begin_state(batch_size=0, ctx=mx_context)
<#else>
                    ${member} = mx.nd.zeros((${tc.join(tc.getLayerVariableMembers("batch_size")[member][0], ", ")},), ctx=mx_context)
</#if>
</#list>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#list>

<#assign instructionCounter = 0>
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
<#list networkInstruction.toUnrollInstruction().resolvedBodies as resolvedBody>
                    <#if networkInstruction.name == "BeamSearch">
                    input = ${tc.join(tc.getStreamInputNames(networkInstruction.body, resolvedBody), ", ")}
                    <#assign depth = tc.getBeamSearchDepth(networkInstruction.toUnrollInstruction())>
                    <#assign width = tc.getBeamSearchWidth(networkInstruction.toUnrollInstruction())>
                    ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]} = applyBeamSearch(input, 0, ${depth}, ${width}, 1.0, ${networkInstruction?index}, input)
                    <#else>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body, resolvedBody), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, resolvedBody), ", ")})
                    <#list resolvedBody.elements as element>
                    <#if element.name == "ArgMax">
                    ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]}, axis=1)
                    </#if>
                    </#list>
                    </#if>
</#list>
<#else>
<#if networkInstruction.body.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body), ", ")})
                    <#list networkInstruction.body.elements as element>
                    <#if element.name == "ArgMax" && tc.architecture.networkInstructions[instructionCounter+1].getName() != "BeamSearch">
                    ${tc.getStreamOutputNames(networkInstruction.body)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body)[0]}, axis=1)
                    </#if>
                    </#list>
<#else>
${tc.include(networkInstruction.body, "PYTHON_INLINE")}
</#if>
</#if>
<#assign instructionCounter = instructionCounter + 1>
</#list>