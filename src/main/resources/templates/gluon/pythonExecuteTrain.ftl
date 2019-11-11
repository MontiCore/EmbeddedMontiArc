                    labels = [batch.label[i].as_in_context(mx_context) for i in range(${tc.architectureOutputs?size?c})]

<#list tc.architectureInputs as input_name>
                    ${input_name} = batch.data[${input_name?index}].as_in_context(mx_context)
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                    ${outputName} = [mx.nd.zeros((train_batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=mx_context) for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((train_batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                    ${member} = mx.nd.zeros((train_batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.constants as constant>
                    ${tc.getName(constant)} = mx.nd.full((train_batch_size, 1,), ${constant.intValue?c}, ctx=mx_context)
</#list>

                    lossList = []

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
<#if tc.isAttentionNetwork()>
                        ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")}, _ = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})
<#else>
                        ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})
</#if>

<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                        ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
<#else>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body, true), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")})

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                    lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                    ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
</#if>
</#list>