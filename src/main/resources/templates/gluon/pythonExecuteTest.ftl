<#-- (c) https://github.com/MontiCore/monticore -->
                        labels = [batch.label[i].as_in_context(mx_context[0]) for i in range(${tc.architectureOutputs?size?c})]
<#assign input_index = 0>
<#if tc.architecture.useDgl>
                        test_input_index = 0
                        graph_ = dgl.batch(test_graph[batch_size*test_batches:min(len(test_graph), batch_size*(test_batches+1))])
</#if>
<#list tc.architectureInputs as input_name>
<#if input_name?index == tc.architectureInputs?seq_index_of(input_name)>
<#if tc.architecture.useDgl>
<#if input_name != 'graph_'>
                        if '${input_name}' in graph_.ndata:
                            ${input_name} = graph_.ndata['${input_name}']
                        elif '${input_name}' in graph_.edata:
                            ${input_name} = graph_.edata['${input_name}']
                        else:
                            ${input_name} = batch.data[test_input_index].as_in_context(mx_context[0])
                            test_input_index += 1
</#if>
<#else>
                        ${input_name} = batch.data[${input_index}].as_in_context(mx_context[0])
</#if>
<#assign input_index++>
</#if>
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                        ${outputName} = [mx.nd.zeros((single_pu_batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=mx_context[0]) for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                        ${tc.getName(output)} = mx.nd.zeros((single_pu_batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context[0])<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                        ${member} = mx.nd.zeros((single_pu_batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context[0])
</#list>

<#list tc.architecture.constants as constant>
                        ${tc.getName(constant)} = mx.nd.full((single_pu_batch_size, 1,), ${constant.intValue?c}, ctx=mx_context[0])
</#list>

                        nd.waitall()

                        lossList = []
                        outputs = []
                        attentionList = []

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                        k = ${tc.getBeamSearchWidth(networkInstruction)}
<#list tc.getUnrollInputNames(networkInstruction, "1") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                        sequences = [([${inputName}], mx.nd.full((single_pu_batch_size, 1,), 1.0, ctx=mx_context[0]), [mx.nd.full((single_pu_batch_size, 64,), 0.0, ctx=mx_context[0])], [mx.nd.full((single_pu_batch_size, 64,), 0.0, ctx=mx_context[0])])]
</#if>
</#list>

                        for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
                            all_candidates = []

                            for seq, score, seqLossList, attention in sequences:
<#list tc.getUnrollInputNames(networkInstruction, "i") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                                ${inputName} = seq[-1]
</#if>
</#list>
                                net_ret = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})

                                ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")} = net_ret[0]
<#if tc.isAttentionNetwork()>
                                attention_ = net_ret[-1][0]
</#if>
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                                out = ${outputName}
                                newLossList = seqLossList + [loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}])]
</#if>
</#list>

                                topk = out.topk(k=k)

                                for top_index in range(len(topk[0])):
                                    j = mx.nd.slice_axis(topk, axis=1, begin=top_index, end=top_index+1)
                                    currentScore = mx.nd.slice_axis(out, axis=1, begin=top_index, end=top_index+1)
                                    newScore = mx.nd.expand_dims(score.squeeze() * currentScore.squeeze(), axis=1)
<#if tc.isAttentionNetwork()>
                                    candidate = (seq + [j],  newScore, newLossList, attention + [attention_])
<#else>
                                    candidate = (seq + [j],  newScore, newLossList, attention + [])
</#if>
                                    all_candidates.append(candidate)

                            ordered = []
                            newSequences = []
                            for batch_entry in range(batch_size):
                                ordered.append([])
                                batchCandidate = [([seq[batch_entry] for seq in candidate[0]], candidate[1][batch_entry], [seq[batch_entry] for seq in candidate[2]], [attention[batch_entry].expand_dims(axis=0) for attention in candidate[3]]) for candidate in all_candidates]
                                ordered[batch_entry] = sorted(batchCandidate, key=lambda tup: tup[1].asscalar())
                                if batch_entry == 0:
                                    newSequences = ordered[batch_entry]
                                else:
                                    newSequences = [([mx.nd.concat(newSequences[sequenceIndex][0][seqIndex], ordered[batch_entry][sequenceIndex][0][seqIndex], dim=0) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                                        mx.nd.concat(newSequences[sequenceIndex][1], ordered[batch_entry][sequenceIndex][1], dim=0),
                                        [mx.nd.concat(newSequences[sequenceIndex][2][lossIndex], ordered[batch_entry][sequenceIndex][2][lossIndex], dim=0) for lossIndex in range(len(newSequences[sequenceIndex][2]))],
                                        [mx.nd.concat(newSequences[sequenceIndex][3][attentionIndex], ordered[batch_entry][sequenceIndex][3][attentionIndex], dim=0) for attentionIndex in range(len(newSequences[sequenceIndex][3]))])
                                        for sequenceIndex in range(len(newSequences))]

                            newSequences = [([newSequences[sequenceIndex][0][seqIndex].expand_dims(axis=1) for seqIndex in range(len(newSequences[sequenceIndex][0]))],
                                newSequences[sequenceIndex][1].expand_dims(axis=1),
                                newSequences[sequenceIndex][2],
                                [newSequences[sequenceIndex][3][attentionIndex] for attentionIndex in range(len(newSequences[sequenceIndex][3]))])
                                for sequenceIndex in range(len(newSequences))]

                            sequences = newSequences[:][:k]

                        for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                            ${outputName} = sequences[0][0][i]
                            outputs.append(${outputName})
                            lossList.append(sequences[0][2][i])
<#if tc.isAttentionNetwork()>
                            attentionList.append(sequences[0][3][i])
</#if>
</#if>
</#list>
<#elseif networkInstruction.body.anyEpisodicLocalAdaptation>
                        for layer_i, layer in enumerate(episodic_layers[${networkInstruction?index}]):
                            if layer.use_local_adaptation:

                                local_adaptation_output = self._networks[${networkInstruction?index}].episodicsubnet0_(${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")})[0]
                                for i in range(1, layer_i):
                                    local_adaptation_output = self._networks[${networkInstruction?index}].episodic_sub_nets[i](*local_adaptation_output)[0]

                                local_adaptation_batch = layer.sample_neighbours(local_adaptation_output, episodic_query_networks[${networkInstruction?index}][layer_i])

                                local_adaptation_data = {}
                                local_adaptation_labels = {}
                                local_adaptation_data[layer_i] = [[local_adaptation_batch[0][i][j].as_in_context(mx_context[0]) for i in range(len(local_adaptation_batch[0]))] for j in range(single_pu_batch_size)]
                                local_adaptation_labels[layer_i] = [[local_adaptation_batch[1][i][j].as_in_context(mx_context[0]) for i in range(len(local_adaptation_batch[1]))] for j in range(single_pu_batch_size)]

                        for local_adaptation_batch_i in range(single_pu_batch_size):

                            self._networks[${networkInstruction?index}].collect_params().load_dict(params[${networkInstruction?index}], ctx=mx_context[0])

                            if len(self._networks[${networkInstruction?index}].collect_params().values()) != 0:
                                if optimizer == "adamw":
                                    local_adaptation_trainer = mx.gluon.Trainer(self._networks[${networkInstruction?index}].collect_params(), AdamW.AdamW(**optimizer_params))
                                else:
                                    local_adaptation_trainer = mx.gluon.Trainer(self._networks[${networkInstruction?index}].collect_params(), optimizer, optimizer_params)

                            for layer_i, layer in enumerate(episodic_layers[${networkInstruction?index}]):
                                if layer.use_local_adaptation:
                                    for gradient_step in range(layer.local_adaptation_gradient_steps):
                                        with autograd.record():
                                            local_adaptation_output = self._networks[${networkInstruction?index}].episodic_sub_nets[layer_i](*(local_adaptation_data[layer_i][local_adaptation_batch_i]))[0]
                                            for i in range(layer_i+1, len(episodic_layers[${networkInstruction?index}])):
                                                local_adaptation_output = self._networks[${networkInstruction?index}].episodic_sub_nets[i](*local_adaptation_output)[0]

                                            curr_param_dict = self._networks[${networkInstruction?index}].collect_params()
                                            curr_params = {}
                                            for param in curr_param_dict:
                                                curr_params[param] = curr_param_dict[param].data()
                                            local_adaptation_loss_list = []
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                                            local_adaptation_loss_list.append(local_adaptation_loss_function(local_adaptation_output[${outputName?index}], local_adaptation_labels[layer_i][local_adaptation_batch_i][${outputName?index}], curr_params, params[${networkInstruction?index}]))
</#if>
</#list>

                                            loss = 0
                                            for element in local_adaptation_loss_list:
                                                loss = loss + element

                                        loss.backward()

                                        if clip_global_grad_norm:
                                                grads = []

                                                for network in self._networks.values():
                                                    grads.extend([param.grad(mx_context) for param in network.collect_params().values()])

                                                gluon.utils.clip_global_norm(grads, clip_global_grad_norm)

                                        local_adaptation_trainer.step(layer.k)
                            outputs = []
                            lossList = []
                            net_ret = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ".take(nd.array([local_adaptation_batch_i], ctx=mx_context[0])), ")}.take(nd.array([local_adaptation_batch_i], ctx=mx_context[0])))
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
                            ${outputName} = net_ret[0][${outputName?index}]
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                            outputs.append(${outputName})
                            lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}][local_adaptation_batch_i]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                            ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
<#else>
                        net_ret = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")})
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
                        ${outputName} = net_ret[0][${outputName?index}]
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        outputs.append(${outputName})
                        lossList.append(loss_function(${outputName}, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                        ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
</#if>
</#list>
