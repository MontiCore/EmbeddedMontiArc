<#list tc.getLayerVariableMembers()?keys as member>
                    ${member} = mx.nd.zeros((batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context)
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
                    <#assign length = tc.getBeamSearchLength(networkInstruction.toUnrollInstruction())>
                    <#assign width = tc.getBeamSearchWidth(networkInstruction.toUnrollInstruction())>
                    ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]} = applyBeamSearch(input, 0, ${length}, ${width}, 1.0, ${networkInstruction?index}, input)
                    <#else>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body, resolvedBody), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, resolvedBody), ", ")})
                    <#if !(tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]?ends_with("_output_"))>
                    outputs.append(${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]})
                    </#if>
                    <#list resolvedBody.elements as element>
                    <#if element.name == "ArgMax">
                    ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]}, axis=1).expand_dims(1)
                    </#if>
                    </#list>
                    </#if>
</#list>
<#else>
<#if networkInstruction.body.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body), ", ")})
                    <#if !(tc.getStreamOutputNames(networkInstruction.body)[0]?ends_with("_output_"))>
                    outputs.append(${tc.getStreamOutputNames(networkInstruction.body)[0]})
                    </#if>
                    <#list networkInstruction.body.elements as element>
                    <#if element.name == "ArgMax" && (tc.architecture.networkInstructions?size <= instructionCounter+1 || tc.architecture.networkInstructions[instructionCounter+1].getName() != "BeamSearch")>
                    ${tc.getStreamOutputNames(networkInstruction.body)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body)[0]}, axis=1).expand_dims(1)
                    </#if>
                    </#list>
<#else>
${tc.include(networkInstruction.body, "PYTHON_INLINE")}
<#if !(tc.getStreamOutputNames(networkInstruction.body)[0]?ends_with("_state_"))>
                    outputs.append(${tc.getStreamOutputNames(networkInstruction.body)[0]})
</#if>
</#if>
</#if>
<#assign instructionCounter = instructionCounter + 1>
</#list>