                if batch_i > 0:
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.replaySubNetworks?has_content>
                    for layer_i, layer in enumerate(replay_layers[${networkInstruction?index}]):
                        if batch_i % layer.replay_interval == 0:
                            replay_batches = layer.sample_memory(batch_size, mx_context)

                            for replay_batch in replay_batches:       
                                labels = [replay_batch[1][i].as_in_context(mx_context) for i in range(${tc.architectureOutputs?size?c})]
                                replay_data = replay_batch[0].as_in_context(mx_context)

                                for gradient_step in range(layer.replay_gradient_steps):
                                    with autograd.record():

                                        nd.waitall()

                                        lossList = []

                                        replay_output = self._networks[${networkInstruction?index}].replay_sub_nets[layer_i](replay_data)[0][0]
                                        for i in range(layer_i+1, len(replay_layers[${networkInstruction?index}])):
                                            replay_output = self._networks[${networkInstruction?index}].replay_sub_nets[i](replay_output)[0][0]

<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                                        lossList.append(loss_function(replay_output, labels[${tc.getIndex(outputName, true)}]))
<#if tc.endsWithArgmax(networkInstruction.body)>
                                        replay_output[${outputName?index}] = mx.nd.argmax(replay_output[${outputName?index}], axis=1).expand_dims(1)
</#if>
</#if>
</#list>
</#if>
</#list>
                pass