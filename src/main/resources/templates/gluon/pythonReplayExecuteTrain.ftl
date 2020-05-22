<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.replaySubNetworks?has_content>
<#if !replayVisitedReplayExecuteTrain??>
                #replay memory computations
                if batch_i > 0:
<#assign replayVisitedReplayExecuteTrain = true>
</#if>
                    for layer_i, layer in enumerate(replay_layers[${networkInstruction?index}]):
                        if batch_i % layer.replay_interval == 0 and layer.use_replay:
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
                                        loss = 0
                                        for element in lossList:
                                            loss = loss + element
                                
                                    loss.backward()
                        
                                    loss_total += loss.sum().asscalar()
                                    global_loss_train += loss.sum().asscalar()

                                    if clip_global_grad_norm:
                                        grads = []

                                        for network in self._networks.values():
                                            grads.extend(
                                                [param.grad(mx_context) for param in network.collect_params().values()])

                                        gluon.utils.clip_global_norm(grads, clip_global_grad_norm)

                                    for trainer in trainers:
                                        trainer.step(batch_size, ignore_stale_grad=True)
</#if>
</#list>
<#if replayVisitedReplayExecuteTrain??>
                    pass
</#if>