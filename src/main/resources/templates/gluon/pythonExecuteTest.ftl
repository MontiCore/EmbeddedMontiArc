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

                    outputs = []
                    attentionList=[]
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.isUnroll()>
                    k = ${tc.getBeamSearchWidth(networkInstruction)}
<#list tc.getUnrollInputNames(networkInstruction, "1") as inputName>
<#if tc.getNameWithoutIndex(inputName) == tc.outputName>
                    sequences = [([${inputName}], 1.0)]
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

                            topk = out.topk(k=k)[0]

                            for j in topk:
                                candidate = (seq + [mx.nd.full((1, 1,), j, ctx=mx_context)], score * out[0][j].asscalar())
                                all_candidates.append(candidate)

                        ordered = sorted(all_candidates, key=lambda tup: tup[1])
                        sequences = ordered[:k]

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