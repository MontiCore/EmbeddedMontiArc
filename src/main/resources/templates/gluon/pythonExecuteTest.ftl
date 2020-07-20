                    labels = [gluon.utils.split_and_load(batch.label[i], ctx_list=mx_context, even_split=False) for i in range(${tc.architectureOutputs?size?c})]
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

                    outputs = []
                    attentionList = []

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    k = ${tc.getBeamSearchWidth(networkInstruction)}
<#list tc.getUnrollInputNames(networkInstruction, "1") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                    sequences = [([${inputName}], mx.nd.full((batch_size, 1,), 1.0, ctx=mx_context[0]), [mx.nd.full((batch_size, 64,), 0.0, ctx=mx_context[0])], [mx.nd.full((batch_size, 64,), 0.0, ctx=mx_context[0])])]
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
<#else>
                    net_ret = [self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), "[i], ")}[i]) for i in range(num_pus)]

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
                    ${outputName} = [net_ret[i][0][${outputName?index}] for i in range(num_pus)]
</#list>

                    losses = []
                    for i in range(num_pus):
                        outputs.append([])
                        lossList = []
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        outputs[i].append(${outputName}[i])
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