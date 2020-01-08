        gen_inputs = gen_net.getInputs()
        qnet_outputs = []
        if self.use_qnet:
            qnet_outputs = q_net.getOutputs()
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
                        generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.randint(low=int(min),
                                                    high=int(max)+1, shape=(batch_size,)+domain[3],
                                                    ctx=mx_context), dtype="float32")

                if namen[-1] in constraint_losses:
                    wierd = 'doNothingYet'
                else:
                    if domain[0] == float:
                        qnet_losses += [mx.gluon.loss.L2Loss()]
                    elif domain[0] == int:
                        qnet_losses += [lambda pred, labels: mx.gluon.loss.SoftmaxCrossEntropyLoss()(pred, labels.reshape(batch_size))]



        for name in gen_inputs:
            if not name in qnet_outputs:
                domain = gen_inputs[name]
                min = domain[1]
                max = domain[2]
                if noise_distribution == "gaussian":
                    generators[name] = lambda domain=domain, min=min, max=max: mx.nd.cast(mx.ndarray.random.normal(noise_distribution_params["mean_value"],
                                                                noise_distribution_params["spread_value"],
                                                                shape=(batch_size,)+domain[3], dtype=domain[0],
                                                                ctx=mx_context), dtype="float32")

        def create_generator_input():
            expected_output_qnet = []
            input_to_gen = []
            for name in gen_inputs:
                if not name in qnet_outputs:
                    input_to_gen += [generators[name]()]
            for name in qnet_outputs:
                expected_output_qnet += [generators[name]()]
                input_to_gen += [generators[name]()]
            return input_to_gen, expected_output_qnet
