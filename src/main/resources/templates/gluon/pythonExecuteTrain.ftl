                    labels = [batch.label[i].as_in_context(mx_context) for i in range(${tc.architectureOutputs?size?c})]

<#list tc.architectureInputs as input_name>
                    ${input_name} = batch.data[${input_name?index}].as_in_context(mx_context)
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                    ${outputName} = [mx.nd.zeros((batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=mx_context) for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                    ${member} = mx.nd.zeros((batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.constants as constant>
                    ${tc.getName(constant)} = mx.nd.full((batch_size, 1,), ${constant.intValue?c}, ctx=mx_context)
</#list>

                    nd.waitall()

                    lossList = []

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
                        net_ret = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})                                                        
                        ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")} = net_ret[0]

<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                        ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
                        if use_teacher_forcing == "True":
                            ${outputName} = mx.nd.expand_dims(labels[${tc.getIndex(outputName, true)}], axis=1)
</#if>
</#list>
<#else>
                    net_ret = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")})
<#if networkInstruction.body.replaySubNetworks?has_content>

                    for i in range(len(replay_layers[${networkInstruction?index}])):
                        replay_store_buffer[${networkInstruction?index}][i] = net_ret[1][i]
</#if>

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
                    ${outputName} = net_ret[0][${outputName?index}]
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                    lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                    ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
</#if>
</#list>