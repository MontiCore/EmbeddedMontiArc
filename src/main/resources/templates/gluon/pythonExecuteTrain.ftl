<#-- (c) https://github.com/MontiCore/monticore -->
                    labels = [gluon.utils.split_and_load(batch.label[i], ctx_list=mx_context, even_split=False) for i in range(${tc.architectureOutputs?size?c})]
<#assign input_index = 0>
<#if tc.architecture.useDgl>
                    train_data_index = 0
                    graph_ = [dgl.batch(train_graph[batch_size*train_batches:min(len(train_graph), batch_size*(train_batches+1))])]
</#if>
<#list tc.architectureInputs as input_name>
<#if input_name?index == tc.architectureInputs?seq_index_of(input_name)>
<#if tc.architecture.useDgl>
<#if input_name != 'graph_'>
                    if '${input_name}' in graph_[0].ndata:
                        ${input_name} = [graph_[0].ndata['${input_name}']]
                    elif '${input_name}' in graph_[0].edata:
                        ${input_name} = [graph_[0].edata['${input_name}']]
                    else:
                        ${input_name} = gluon.utils.split_and_load(batch.data[train_data_index], ctx_list=mx_context, even_split=False)
                        train_data_index += 1
                    if not multi_graph:
                        ${input_name} = ${input_name}[0]
</#if>
<#else>
                    ${input_name} = gluon.utils.split_and_load(batch.data[${input_index}], ctx_list=mx_context, even_split=False)
<#assign input_index++>
</#if>
</#if>
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                    ${outputName} = [[mx.nd.zeros((single_pu_batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=context) for context in mx_context] for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = [mx.nd.zeros((single_pu_batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=context) for context in mx_context]<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                    ${member} = [mx.nd.zeros((single_pu_batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=context) for context in mx_context]
</#list>

<#list tc.architecture.constants as constant>
                    ${tc.getName(constant)} = [mx.nd.full((single_pu_batch_size, 1,), ${constant.intValue?c}, ctx=context) for context in mx_context]
</#list>

                    nd.waitall()
                    lossList = []
                    for i in range(num_pus):
                        lossList.append([])

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    for j in range(num_pus):
                        for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
                            ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), "[j], ")}[j] = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), "[j], ")}[j])[0]
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                            lossList[j].append(loss_function(${outputName}[j], labels[${tc.getIndex(outputName, true)}][j]))
</#if>
</#list>
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>								
<#if tc.endsWithArgmax(networkInstruction.body)>
                            ${outputName}[j] = mx.nd.argmax(${outputName}[j], axis=1).expand_dims(1) 
</#if>
                            if use_teacher_forcing == "True":
                                ${outputName}[j] = mx.nd.expand_dims(labels[${tc.getIndex(outputName, true)}][j], axis=1)
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
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                    if (train_mask is not None or test_mask is not None) and epoch == 0:
                        train_mask = self.get_mask_array(${outputName}[0].shape[0], train_mask)
                        test_mask = self.get_mask_array(${outputName}[0].shape[0], test_mask)
<#if tc.architecture.useDgl>
                    elif not multi_graph and epoch == 0:
                        train_mask = graph_[0].ndata['train_mask']
                        test_mask = graph_[0].ndata['test_mask']
</#if>
                    if train_mask is not None:
                        outputs = [${outputName}[0]]
                        [lossList[i].append(loss_function(mx.nd.squeeze(${outputName}[i]), mx.nd.squeeze(labels[${tc.getIndex(outputName, true)}][i]), mx.nd.expand_dims(train_mask, 1)).sum() / train_mask.sum().asscalar()) for i in range(num_pus)]
                    else:
                        [lossList[i].append(loss_function(${outputName}[i], labels[${tc.getIndex(outputName, true)}][i])) for i in range(num_pus)]

</#if>
</#list>

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
<#if tc.endsWithArgmax(networkInstruction.body)>
                    ${outputName} = [mx.nd.argmax(${outputName}[i], axis=1).expand_dims(1) for i in range(num_pus)]
</#if>
</#if>
</#list>
</#if>
</#list>
