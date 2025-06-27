<#-- (c) https://github.com/MontiCore/monticore -->
        gen_inputs = self._net_creator_gen.getInputs()
        dis_inputs = self._net_creator_dis.getInputs()

        qnet_outputs = []
        if self.use_qnet:
            qnet_outputs = self._net_creator_qnet.getOutputs()
            qnet_losses = []
        generators = {}
        if self.use_qnet:
            for name in qnet_outputs:
                domain = gen_inputs[name]
                min = domain[1]
                max = domain[2]
                if name[:-1] in constraint_distributions:
                    dist_dict = constraint_distributions[name[:-1]]
                    dist_name = dist_dict['name']
                    if dist_name is "gaussian":
                        generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.normal(dist_dict["mean_value"],
                                                                    dist_dict["spread_value"],
                                                                    shape=(batch_size,)+domain[3], dtype=domain[0],
                                                                    ctx=mx_context), dtype="float32")
                else:
                    if domain[0] == float:
                        generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.uniform(min,max,
                                                    shape=(batch_size,)+domain[3],
                                                    dtype=domain[0], ctx=mx_context,), dtype="float32")
                    elif domain[0] == int:
                        generators[name] = lambda domain=domain, min=min, max=max:mx.ndarray.one_hot(mx.ndarray.random.randint(low=0,
                                                    high=int(max-min)+1, shape=(batch_size,), dtype=int,
                                                    ctx=mx_context), depth=int(max-min)+1, on_value=1).reshape((batch_size,)+domain[3])

                if name[-1] in constraint_losses:
                    loss_dict = constraint_losses[name[:-1]]
                    loss = loss_dict['name']
                    margin = loss_dict['margin'] if 'margin' in loss_dict else 1.0
                    sparseLabel = loss_dict['sparse_label'] if 'sparse_label' in loss_dict else True
                    ignore_indices = [loss_dict['ignore_indices']] if 'ignore_indices' in loss_dict else []
                    fromLogits = loss_dict['from_logits'] if 'from_logits' in loss_dict else False

                    if loss == 'softmax_cross_entropy':
                        qnet_losses += [mx.gluon.loss.SoftmaxCrossEntropyLoss(from_logits=fromLogits, sparse_label=sparseLabel)]
                    elif loss == 'softmax_cross_entropy_ignore_indices':
                        qnet_losses += [SoftmaxCrossEntropyLossIgnoreIndices(ignore_indices=ignore_indices, from_logits=fromLogits, sparse_label=sparseLabel)]
                    elif loss == 'sigmoid_binary_cross_entropy':
                        qnet_losses += [mx.gluon.loss.SigmoidBinaryCrossEntropyLoss(from_sigmoid=True)]
                    elif loss == 'cross_entropy':
                        qnet_losses += [CrossEntropyLoss(sparse_label=sparseLabel)]
                    elif loss == 'l2':
                        qnet_losses += [mx.gluon.loss.L2Loss()]
                    elif loss == 'l1':
                        qnet_losses += [mx.gluon.loss.L2Loss()]
                    elif loss == 'log_cosh':
                        qnet_losses += [LogCoshLoss()]
                    else:
                        logging.error("Invalid loss parameter for constraint:" + name[:-1] + ".")
                else:
                    if domain[0] == float:
                        qnet_losses += [mx.gluon.loss.L2Loss()]
                    elif domain[0] == int:
                        qnet_losses += [lambda pred, labels: mx.gluon.loss.SoftmaxCrossEntropyLoss(sparse_label=False)(pred, labels)]

        for name in gen_inputs:
            if name == noise_input + "_":
                domain = gen_inputs[name]
                min = domain[1]
                max = domain[2]
                if noise_distribution == "gaussian":
                    generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.normal(noise_distribution_params["mean_value"],
                                                                noise_distribution_params["spread_value"],
                                                                shape=(batch_size,)+domain[3], dtype=domain[0],
                                                                ctx=mx_context), dtype="float32")
                elif noise_distribution == "uniform":
                    generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.uniform(low=min,
                                                                high=max, shape=(batch_size,)+domain[3], dtype=domain[0],
                                                                ctx=mx_context), dtype="float32")

        def create_generator_input(cur_batch):
            expected_qnet_output = []
            gen_input = []

            for name in gen_inputs:
                if name in traindata_to_index.keys():
                    gen_input += [cur_batch.data[traindata_to_index[name]].as_in_context(mx_context)]
                elif name in qnet_outputs:
                    value = generators[name]()
                    expected_qnet_output += [value]
                    gen_input += [value]
                else:
                    gen_input += [generators[name]()]
            return gen_input, expected_qnet_output

        def create_discriminator_input(cur_batch):
            conditional_input = []
            for name in gen_inputs:
                if name in traindata_to_index.keys():
                    conditional_input += [cur_batch.data[traindata_to_index[name]].as_in_context(mx_context)]
            return conditional_input
