<#-- (c) https://github.com/MontiCore/monticore -->
<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.episodicSubNetworks?has_content>
<#if !visitedEpisodicExecuteTrain??>
                #episodic replay memory computations
                if batch_i > 0:
<#assign visitedEpisodicExecuteTrain = true>
</#if>
                    for layer_i, layer in enumerate(episodic_layers[${networkInstruction?index}]):
                        if batch_i % layer.replay_interval == 0 and layer.use_replay:
                            episodic_batches = layer.sample_memory(batch_size)

                            for episodic_batch in episodic_batches:
                                labels = [gluon.utils.split_and_load(episodic_batch[1][i], ctx_list=mx_context, even_split=False) for i in range(${tc.architectureOutputs?size?c})]
                                episodic_data = [[] for i in range(num_pus)]
                                for i in range(len(episodic_batch[0])):
                                    tmp_data = gluon.utils.split_and_load(episodic_batch[0][i], ctx_list=mx_context, even_split=False)
                                    [episodic_data[j].append(tmp_data[j]) for j in range(num_pus)]

                                for gradient_step in range(layer.replay_gradient_steps):
                                    with autograd.record():

                                        episodic_output = [self._networks[${networkInstruction?index}].episodic_sub_nets[layer_i](*(episodic_data[i]))[0] for i in range(num_pus)]
                                        for i in range(layer_i+1, len(episodic_layers[${networkInstruction?index}])):
                                            episodic_output = [self._networks[${networkInstruction?index}].episodic_sub_nets[i](*(episodic_output[j]))[0] for j in range(num_pus)]

                                        losses = []
                                        for i in range(num_pus):
                                            lossList = []
<#list tc.getStreamOutputNames(networkInstruction.body, true) as outputName>
<#if tc.getNameWithoutIndex(outputName) == tc.outputName>
                                            lossList.append(loss_function(episodic_output[i][${tc.getIndex(outputName, true)}], labels[${tc.getIndex(outputName, true)}][i]))
</#if>
</#list>
                                            losses.append(0)
                                            for element in lossList:
                                                losses[i] = losses[i] + element

                                    for loss in losses:
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
<#if visitedEpisodicExecuteTrain??>
                    pass
</#if>
