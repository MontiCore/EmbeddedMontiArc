<#list tc.getLayerVariableMembers("batch_size")?keys as member>
                    ${member} = mx.nd.zeros((${tc.join(tc.getLayerVariableMembers("batch_size")[member], ", ")},), ctx=mx_context)
</#list>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#list>

                    lossList = []
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
<#list networkInstruction.toUnrollInstruction().resolvedBodies as resolvedBody>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body, resolvedBody), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, resolvedBody), ", ")})
                    lossList.append(loss_function(${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]}, ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]}label))
                    <#list resolvedBody.elements as element>
                    <#if element.name == "ArgMax">
                    ${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body, resolvedBody)[0]}, axis=1)
                    </#if>
                    </#list>
</#list>
<#else>
<#if networkInstruction.body.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body), ", ")})
                    <#if tc.getStreamOutputNames(networkInstruction.body)[0] != "encoder_output_">
                    lossList.append(loss_function(${tc.getStreamOutputNames(networkInstruction.body)[0]}, ${tc.getStreamOutputNames(networkInstruction.body)[0]}label))
                    </#if>
                    <#list networkInstruction.body.elements as element>
                    <#if element.name == "ArgMax">
                    ${tc.getStreamOutputNames(networkInstruction.body)[0]} = mx.nd.argmax(${tc.getStreamOutputNames(networkInstruction.body)[0]}, axis=1)
                    </#if>
                    </#list>
<#else>
${tc.include(networkInstruction.body, "PYTHON_INLINE")}
</#if>
</#if>
</#list>