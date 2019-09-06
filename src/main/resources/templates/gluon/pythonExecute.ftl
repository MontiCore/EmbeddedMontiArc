<#list tc.getLayerVariableMembers("batch_size")?keys as member>
                    ${member} = mx.nd.zeros((${tc.join(tc.getLayerVariableMembers("batch_size")[member], ", ")},), ctx=mx_context)
</#list>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body), ", ")})
<#else>
${tc.include(networkInstruction.body, "PYTHON_INLINE")}
</#if>
</#list>