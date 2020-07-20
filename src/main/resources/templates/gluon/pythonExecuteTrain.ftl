                    labels = [gluon.utils.split_and_load(batch.label[i], ctx_list=mx_context, even_split=False) for i in range(1)]

<#list tc.architectureInputs as input_name>
                    ${input_name} = gluon.utils.split_and_load(batch.data[${input_name?index}], ctx_list=mx_context, even_split=False)
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                    #${outputName} = [mx.nd.zeros((batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=mx_context) for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                    #${tc.getName(output)} = mx.nd.zeros((batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                    #${member} = mx.nd.zeros((batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.constants as constant>
                    #${tc.getName(constant)} = mx.nd.full((batch_size, 1,), ${constant.intValue?c}, ctx=mx_context)
</#list>

                    nd.waitall()

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
                        net_ret = [self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), "[j], ")}[j]) for j in range(num_pus)]                                                       
                        ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")} = [net_ret[j][0] for j in range(num_pus)]

                        losses = []
                        for j in range(num_pus):
                            lossList = []
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                            lossList.append(loss_function(${outputName}[j], labels[${tc.getIndex(outputName, true)}][j]))
</#if>
</#list>
                            losses.append(0)
                            for element in lossList:
                                losses[j] = losses[j] + element

<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>								
<#if tc.endsWithArgmax(networkInstruction.body)>
                        ${outputName} = [mx.nd.argmax(${outputName}[j], axis=1).expand_dims(1), for j in range(num_pus)]
</#if>
                        if use_teacher_forcing == "True":
                            ${outputName} = [mx.nd.expand_dims(labels[${tc.getIndex(outputName, true)}][j], axis=1) for j in range(num_pus)]
</#if>
</#list>
<#else>
                    net_ret = [self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), "[i], ")}[i]) for i in range(num_pus)]
<#if networkInstruction.body.episodicSubNetworks?has_content>

                    for i in range(len(episodic_layers[${networkInstruction?index}])):
                        temp_buffer_data = []
                        for j in range(num_pus):
                            temp_buffer_data.append(net_ret[j][1][i])
                        episodic_store_buffer[${networkInstruction?index}][i] = temp_buffer_data						
</#if>

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
                    ${outputName} = [net_ret[i][0][${outputName?index}] for i in range(num_pus)]
</#list>

                    losses = []
                    for i in range(num_pus):
                        lossList = []
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        lossList.append(loss_function(${outputName}[i], labels[${tc.getIndex(outputName, true)}][i]))						
</#if>
</#list>
                        losses.append(0)
                        for element in lossList:
                            losses[i] = losses[i] + element	

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
<#if tc.endsWithArgmax(networkInstruction.body)>
                    ${outputName} = [mx.nd.argmax(${outputName}[i], axis=1).expand_dims(1), for i in range(num_pus)]
</#if>
</#if>
</#list>
</#if>
</#list>