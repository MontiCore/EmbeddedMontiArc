                    labels = [batch.label[i].as_in_context(mx_context) for i in range(${tc.architectureOutputs?size?c})]

<#list tc.architectureInputs as input_name>
                    ${input_name} = batch.data[${input_name?index}].as_in_context(mx_context)
</#list>

<#if tc.architectureOutputSymbols?size gt 1>
<#assign outputName = tc.getNameWithoutIndex(tc.getName(tc.architectureOutputSymbols[0]))>
                    ${outputName} = [mx.nd.zeros((test_batch_size, ${tc.join(tc.architectureOutputSymbols[0].ioDeclaration.type.dimensions, ", ")},), ctx=mx_context) for i in range(${tc.architectureOutputs?size?c})]
<#else>
<#list tc.architectureOutputSymbols as output>
                    ${tc.getName(output)} = mx.nd.zeros((test_batch_size, ${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)<#sep>,
</#list>
</#if>

<#list tc.getLayerVariableMembers()?keys as member>
                    ${member} = mx.nd.zeros((test_batch_size, ${tc.join(tc.cutDimensions(tc.getLayerVariableMembers()[member]), ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.constants as constant>
                    ${tc.getName(constant)} = mx.nd.full((test_batch_size, 1,), ${constant.intValue?c}, ctx=mx_context)
</#list>

                    nd.waitall()

                    outputs = []
                    attentionList=[]
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    k = ${tc.getBeamSearchWidth(networkInstruction)}
<#list tc.getUnrollInputNames(networkInstruction, "1") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                    sequences = [([${inputName}], mx.nd.full((batch_size, 1,), 1.0, ctx=mx_context))]
</#if>
</#list>

                    for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
                        all_candidates = []

                        for seq, score in sequences:
<#list tc.getUnrollInputNames(networkInstruction, "i") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                            ${inputName} = seq[-1]
</#if>
</#list>
<#if tc.isAttentionNetwork()>
                            ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")}, attention_ = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})
                            attentionList.append(attention_)
<#else>
                            ${tc.join(tc.getUnrollOutputNames(networkInstruction, "i"), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getUnrollInputNames(networkInstruction, "i"), ", ")})
</#if>
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                            out = ${outputName}
</#if>
</#list>

                            topk = out.topk(k=k)

                            for top_index in range(len(topk[0])):
                                j = mx.nd.slice_axis(topk, axis=1, begin=top_index, end=top_index+1)
                                currentScore = mx.nd.slice_axis(out, axis=1, begin=top_index, end=top_index+1)
                                newScore = mx.nd.expand_dims(score.squeeze() * currentScore.squeeze(), axis=1)
                                candidate = (seq + [j],  newScore)
                                all_candidates.append(candidate)

                        ordered = []
                        newSequences = []
                        for batch_entry in range(batch_size):
                            ordered.append([])
                            batchCandidate = [([y[batch_entry] for y in x[0]], x[1][batch_entry]) for x in all_candidates]
                            ordered[batch_entry] = sorted(batchCandidate, key=lambda tup: tup[1].asscalar())
                            if batch_entry == 0:
                                newSequences = ordered[batch_entry]
                            elif batch_entry < (batch_size -1):
                                newSequences = [([mx.nd.concat(newSequences[x][0][y], ordered[batch_entry][x][0][y], dim=0) for y in range(len(newSequences[x][0]))], mx.nd.concat(newSequences[x][1], ordered[batch_entry][x][1], dim=0)) for x in range(len(newSequences))]
                            # expand dims only once
                            else:
                                newSequences = [([mx.nd.concat(newSequences[x][0][y], ordered[batch_entry][x][0][y], dim=0).expand_dims(axis=1) for y in range(len(newSequences[x][0]))], mx.nd.concat(newSequences[x][1], ordered[batch_entry][x][1], dim=0).expand_dims(axis=1)) for x in range(len(newSequences))]

                        sequences = newSequences[:][:k]

                    for i in range(1, ${tc.getBeamSearchMaxLength(networkInstruction)}):
<#list tc.getUnrollOutputNames(networkInstruction, "i") as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                        ${outputName} = sequences[0][0][i]
                        outputs.append(${outputName})
</#if>
</#list>
<#else>
                    ${tc.join(tc.getStreamOutputNames(networkInstruction.body, true), ", ")} = self._networks[${networkInstruction?index}](${tc.join(tc.getStreamInputNames(networkInstruction.body, true), ", ")})

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                    outputs.append(${outputName})
<#if tc.endsWithArgmax(networkInstruction.body)>
                    ${outputName} = mx.nd.argmax(${outputName}, axis=1).expand_dims(1)
</#if>
</#if>
</#list>
</#if>
</#list>