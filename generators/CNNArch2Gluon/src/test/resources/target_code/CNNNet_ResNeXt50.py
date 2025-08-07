# (c) https://github.com/MontiCore/monticore
import mxnet as mx
import numpy as np
import math
import os
import abc
import warnings
import sys
from mxnet import gluon, nd



class ZScoreNormalization(gluon.HybridBlock):
    def __init__(self, data_mean, data_std, **kwargs):
        super(ZScoreNormalization, self).__init__(**kwargs)
        with self.name_scope():
            self.data_mean = self.params.get('data_mean', shape=data_mean.shape,
                init=mx.init.Constant(data_mean.asnumpy()), differentiable=False)
            self.data_std = self.params.get('data_std', shape=data_mean.shape,
                init=mx.init.Constant(data_std.asnumpy()), differentiable=False)

    def hybrid_forward(self, F, x, data_mean, data_std):
        x = F.broadcast_sub(x, data_mean)
        x = F.broadcast_div(x, data_std)
        return x


class Padding(gluon.HybridBlock):
    def __init__(self, padding, **kwargs):
        super(Padding, self).__init__(**kwargs)
        with self.name_scope():
            self.pad_width = padding

    def hybrid_forward(self, F, x):
        x = F.pad(data=x,
            mode='constant',
            pad_width=self.pad_width,
            constant_value=0)
        return x

class NoNormalization(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(NoNormalization, self).__init__(**kwargs)

    def hybrid_forward(self, F, x):
        return x

class Reshape(gluon.HybridBlock):
    def __init__(self, shape, **kwargs):
        super(Reshape, self).__init__(**kwargs)
        with self.name_scope():
            self.shape = shape

    def hybrid_forward(self, F, x):
        return F.reshape(data=x, shape=self.shape)


class CustomRNN(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomRNN, self).__init__(**kwargs)
        with self.name_scope():
            self.rnn = gluon.rnn.RNN(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                     bidirectional=bidirectional, activation='tanh', layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.rnn(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)


class CustomLSTM(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomLSTM, self).__init__(**kwargs)
        with self.name_scope():
            self.lstm = gluon.rnn.LSTM(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                       bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0, state1):
        output, [state0, state1] = self.lstm(data, [F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)])
        return output, F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)


class CustomGRU(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomGRU, self).__init__(**kwargs)
        with self.name_scope():
            self.gru = gluon.rnn.GRU(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                     bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.gru(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)

    
class DotProductSelfAttention(gluon.HybridBlock):
    def __init__(self,
                 scale_factor,
                 num_heads,
                 dim_model,
                 dim_keys,
                 dim_values,
                 use_proj_bias,
                 use_mask,
                 **kwargs):
        super(DotProductSelfAttention, self).__init__(**kwargs)
        with self.name_scope():
            self.num_heads = num_heads
            self.dim_model = dim_model
            self.use_proj_bias = use_proj_bias
            self.use_mask = use_mask

            if dim_keys == -1:
                self.dim_keys = int(dim_model / self.num_heads)
            else:
                self.dim_keys = dim_keys
            if dim_values == -1:
                self.dim_values = int(dim_model / self.num_heads)
            else:
                self.dim_values = dim_values
    
            if scale_factor == -1:
                self.scale_factor = math.sqrt(self.dim_keys)
            else:
                self.scale_factor = scale_factor

            self.proj_q = gluon.nn.Dense(self.num_heads*self.dim_keys, use_bias=self.use_proj_bias, flatten=False)
            self.proj_k = gluon.nn.Dense(self.num_heads*self.dim_keys, use_bias=self.use_proj_bias, flatten=False)
            self.proj_v = gluon.nn.Dense(self.num_heads*self.dim_values, use_bias=self.use_proj_bias, flatten=False)
            self.proj_o = gluon.nn.Dense(self.dim_model, use_bias=self.use_proj_bias, flatten=False)

    def hybrid_forward(self, F, queries, keys, values, *args, **kwargs):

        queries = F.Reshape(queries, shape=(0, 0,-1))
        keys = F.Reshape(queries, shape=(0, 0, -1))
        values = F.Reshape(queries, shape=(0, 0, -1))
    
        head_queries = self.proj_q(queries)
        head_keys = self.proj_k(keys)
        head_values = self.proj_v(values)

        head_queries = F.reshape(head_queries, shape=(0, 0, self.num_heads, -1))
        head_queries = F.transpose(head_queries, axes=(0,2,1,3))
        head_queries = F.reshape(head_queries, shape=(-1, 0, 0), reverse=True)

        head_keys = F.reshape(head_keys, shape=(0, 0, self.num_heads, -1))
        head_keys = F.transpose(head_keys, axes=(0,2,1,3))
        head_keys = F.reshape(head_keys, shape=(-1, 0, 0), reverse=True)

        score = F.batch_dot(head_queries, head_keys, transpose_b=True)
        score = score * self.scale_factor

        if self.use_mask:
            seqs = F.contrib.arange_like(score, axis=1)
            zeros = F.zeros_like(seqs)
            zeros = F.reshape(zeros, shape=(1, -1))
            mask = args[0]
            mask = F.reshape(mask, shape=(-1, 1))
            mask = F.broadcast_add(mask, zeros)
            mask = F.expand_dims(mask, axis=1)
            mask = F.broadcast_axis(mask, axis=1, size=self.num_heads)
            mask = mask.reshape(shape=(-1, 0), reverse=True)
            mask = F.cast(mask, dtype='int32')
            weights = F.softmax(score, mask, use_length=self.use_mask)
        else:
            weights = F.softmax(score)

        head_values = F.reshape(head_values, shape=(0, 0, self.num_heads, -1))
        head_values = F.transpose(head_values, axes=(0,2,1,3))
        head_values = F.reshape(head_values, shape=(-1, 0, 0), reverse=True)

        ret = F.batch_dot(weights, head_values)
        ret = F.reshape(ret, shape=(-1, self.num_heads, 0, 0), reverse=True)
        ret = F.transpose(ret, axes=(0, 2, 1, 3))
        ret = F.reshape(ret, shape=(0, 0, -1))

        ret = self.proj_o(ret)

        return ret

    
class EpisodicReplayMemoryInterface(gluon.HybridBlock):
    __metaclass__ = abc.ABCMeta

    def __init__(self, use_replay, replay_interval, replay_batch_size, replay_steps, replay_gradient_steps, use_local_adaptation, local_adaptation_gradient_steps, k, **kwargs):
        super(EpisodicReplayMemoryInterface, self).__init__(**kwargs)

        self.use_replay = use_replay
        self.replay_interval = replay_interval
        self.replay_batch_size = replay_batch_size
        self.replay_steps = replay_steps
        self.replay_gradient_steps = replay_gradient_steps

        self.use_local_adaptation = use_local_adaptation
        self.local_adaptation_gradient_steps = local_adaptation_gradient_steps
        self.k = k

    @abc.abstractmethod
    def store_samples(self, data, y, query_network, store_prob, mx_context):
        pass

    @abc.abstractmethod
    def sample_memory(self, batch_size, mx_context):
        pass

    @abc.abstractmethod
    def sample_neighbours(self, data, query_network):
        pass

    @abc.abstractmethod
    def get_query_network(self, mx_context):
        pass   

    @abc.abstractmethod
    def save_memory(self, path):
        pass

    @abc.abstractmethod
    def load_memory(self, path):
        pass
    
#Memory layer
class LargeMemory(gluon.HybridBlock):
    def __init__(self, 
                 sub_key_size, 
                 query_size, 
                 query_act,
                 k, 
                 num_heads,
                 values_dim,
                 **kwargs):
        super(LargeMemory, self).__init__(**kwargs)
        with self.name_scope():
            #Memory parameters
            self.k = k
            self.num_heads = num_heads
            self.query_act = query_act
            self.query_size = query_size
            self.num_heads = num_heads
    
            #Batch norm sub-layer
            self.batch_norm = gluon.nn.BatchNorm()

            #Memory sub-layer
            self.sub_key_size = sub_key_size
            sub_key_shape = (self.num_heads, self.sub_key_size, int(query_size[-1] / 2))

            if values_dim == -1:
                values_shape = (self.sub_key_size * self.sub_key_size, self.query_size[-1])
            else:
                values_shape = (self.sub_key_size*self.sub_key_size, values_dim)

            self.sub_keys1 = self.params.get("sub_keys1", shape=sub_key_shape, differentiable=True)
            self.sub_keys2 = self.params.get("sub_keys2", shape=sub_key_shape, differentiable=True)
            self.values = self.params.get("values", shape=values_shape, differentiable=True)
            self.label_memory = nd.array([])

            self.get_query_network()
                        
    def hybrid_forward(self, F, x, sub_keys1, sub_keys2, values):
        x = self.batch_norm(x)

        x = F.reshape(x, shape=(0, -1))

        q = self.query_network(x)

        q = F.reshape(q, shape=(0, self.num_heads, -1))

        q_split = F.split(q, num_outputs=2, axis=-1)

        q1 = F.split(q_split[0], num_outputs=self.num_heads, axis=1)
        q2 = F.split(q_split[1], num_outputs=self.num_heads, axis=1)
        sub_keys1_resh = F.split(sub_keys1, num_outputs=self.num_heads, axis=0, squeeze_axis=True)
        sub_keys2_resh = F.split(sub_keys2, num_outputs=self.num_heads, axis=0, squeeze_axis=True)
        if self.num_heads == 1:
            q1 = [q1]
            q2 = [q2]
            sub_keys1_resh = [sub_keys1_resh ]
            sub_keys2_resh = [sub_keys2_resh ]

        q1_dist = F.dot(q1[0], sub_keys1_resh[0], transpose_b=True)
        q2_dist = F.dot(q2[0], sub_keys2_resh[0], transpose_b=True)
        for h in range(1, self.num_heads):
           q1_dist = F.concat(q1_dist, F.dot(q1[0], sub_keys1_resh[h], transpose_b=True), dim=1)
           q2_dist = F.concat(q2_dist, F.dot(q2[0], sub_keys1_resh[h], transpose_b=True), dim=1)

        i1 = F.topk(q1_dist, k=self.k, ret_typ="indices")
        i2 = F.topk(q2_dist, k=self.k, ret_typ="indices")

        i1 = F.split(i1, num_outputs=self.num_heads, axis=1)
        i2 = F.split(i2, num_outputs=self.num_heads, axis=1)
        sub_keys1 = F.split(sub_keys1, num_outputs=self.num_heads, axis=0, squeeze_axis=True)
        sub_keys2 = F.split(sub_keys2, num_outputs=self.num_heads, axis=0, squeeze_axis=True)
        if self.num_heads == 1:
            i1 = [i1]
            i2 = [i2]
            sub_keys1 = [sub_keys1]
            sub_keys2 = [sub_keys2]

        k1 = F.take(sub_keys1[0], i1[0])
        k2 = F.take(sub_keys2[0], i2[0])
        for h in range(1, self.num_heads):
            k1 = F.concat(k1, F.take(sub_keys1[h], i1[h]), dim=1)
            k2 = F.concat(k2, F.take(sub_keys2[h], i2[h]), dim=1)

        k1 = F.tile(k1, (1, 1, self.k, 1))
        k2 = F.repeat(k2, self.k, 2)
        c_cart = F.concat(k1, k2, dim=3)

        q = F.reshape(q, shape=(-1,0), reverse=True)
        q = F.reshape(q, shape=(0, 1, -1))
        c_cart = F.reshape(c_cart, shape=(-1, 0, 0), reverse=True)

        k_dist = F.batch_dot(q, c_cart, transpose_b=True) #F.contrib.foreach(loop_batch_dot, [q, c_cart], init_states=state_batch_dist)
        k_dist = F.reshape(k_dist, shape=(0, -1))

        i = F.topk(k_dist, k=self.k, ret_typ="both")

        w = F.softmax(i[0])
        w = F.reshape(w, shape=(0,1,-1))
        vi = F.take(values, i[1])
        aggr_value = F.batch_dot(w, vi) #F.contrib.foreach(loop_batch_dot, [w, vi], init_states=state_batch_dist)

        ret = F.reshape(aggr_value, shape=(-1, self.num_heads, 0), reverse=True)
        one_vec = F.ones((1, 1, self.num_heads))
        one_vec = F.broadcast_like(one_vec, ret, lhs_axes=0, rhs_axes=0)
        ret = F.batch_dot(one_vec, ret)
        ret = F.reshape(ret, shape=(-1, 0), reverse=True)

        return ret

    def get_query_network(self):
        if hasattr(self, 'query_network'):
            return self.query_network
        else:
            self.query_network = gluon.nn.HybridSequential()
            for size in self.query_size:
                if self.query_act == "linear":
                    self.query_network.add(gluon.nn.Dense(units=self.num_heads*size, flatten=False))
                else:
                    self.query_network.add(gluon.nn.Dense(units=self.num_heads*size, activation=self.query_act, flatten=False))
            return self.query_network


#EpisodicMemory layer
class EpisodicMemory(EpisodicReplayMemoryInterface):
    def __init__(self,
                 replay_interval,
                 replay_batch_size,
                 replay_steps,
                 replay_gradient_steps,
                 store_prob,
                 max_stored_samples,
                 memory_replacement_strategy,
                 use_replay,
                 use_local_adaptation,
                 local_adaptation_gradient_steps,
                 k,
                 query_net_dir,
                 query_net_prefix,
                 query_net_num_inputs,
                 **kwargs):
        super(EpisodicMemory, self).__init__(use_replay, replay_interval, replay_batch_size, replay_steps, replay_gradient_steps, use_local_adaptation, local_adaptation_gradient_steps, k, **kwargs)
        with self.name_scope():
            #Replay parameters
            self.store_prob = store_prob
            self.max_stored_samples = max_stored_samples
            self.memory_replacement_strategy = memory_replacement_strategy

            self.query_net_dir = query_net_dir
            self.query_net_prefix = query_net_prefix
            self.query_net_num_inputs = query_net_num_inputs
    
            #Memory
            self.key_memory = nd.array([])
            self.value_memory = nd.array([])
            self.label_memory = nd.array([])

    def hybrid_forward(self, F, *args):
        #propagate the input as the rest is only used for replay
        return [args, []]

    def store_samples(self, data, y, query_network, store_prob, context):
        if not (self.memory_replacement_strategy == "no_replacement" and self.max_stored_samples != -1 and self.key_memory.shape[0] >= self.max_stored_samples):
            num_pus = len(data)
            sub_batch_sizes = [data[i][0][0].shape[0] for i in range(num_pus)]
            num_inputs = len(data[0][0])
            num_outputs = len(y)
            mx_context = context[0]

            if len(self.key_memory) == 0:
                self.key_memory = nd.empty(0, ctx=mx.cpu())
                self.value_memory = []
                self.label_memory = []#nd.empty((num_outputs, 0), ctx=mx.cpu())

            ind = [nd.sample_multinomial(store_prob, sub_batch_sizes[i]).as_in_context(mx_context) for i in range(num_pus)]

            max_inds = [nd.max(ind[i]) for i in range(num_pus)]
            if any(max_inds):
                to_store_values = []
                for i in range(num_inputs):
                    tmp_values = []
                    for j in range(0, num_pus):
                        if max_inds[j]:
                            if isinstance(tmp_values, list):
                                tmp_values = nd.contrib.boolean_mask(data[j][0][i].as_in_context(mx_context), ind[j])
                            else:
                                tmp_values = nd.concat(tmp_values, nd.contrib.boolean_mask(data[j][0][i].as_in_context(mx_context), ind[j]), dim=0)
                    to_store_values.append(tmp_values)

                to_store_labels = []
                for i in range(num_outputs):
                    tmp_labels = []
                    for j in range(0, num_pus):
                        if max_inds[j]:
                            if isinstance(tmp_labels, list):
                                tmp_labels = nd.contrib.boolean_mask(y[i][j].as_in_context(mx_context), ind[j])
                            else:
                                tmp_labels = nd.concat(tmp_labels, nd.contrib.boolean_mask(y[i][j].as_in_context(mx_context), ind[j]), dim=0)
                    to_store_labels.append(tmp_labels)

                to_store_keys = query_network(*to_store_values[0:self.query_net_num_inputs])

                if self.key_memory.shape[0] == 0:
                    self.key_memory = to_store_keys.as_in_context(mx.cpu())
                    for i in range(num_inputs):
                        self.value_memory.append(to_store_values[i].as_in_context(mx.cpu()))
                    for i in range(num_outputs):
                        self.label_memory.append(to_store_labels[i].as_in_context(mx.cpu()))
                elif self.memory_replacement_strategy == "replace_oldest" and self.max_stored_samples != -1 and self.key_memory.shape[0] >= self.max_stored_samples:
                    num_to_store = to_store_keys.shape[0]
                    self.key_memory = nd.concat(self.key_memory[num_to_store:], to_store_keys.as_in_context(mx.cpu()), dim=0)
                    for i in range(num_inputs):
                        self.value_memory[i] = nd.concat(self.value_memory[i][num_to_store:], to_store_values[i].as_in_context(mx.cpu()), dim=0)
                    for i in range(num_outputs):
                        self.label_memory[i] = nd.concat(self.label_memory[i][num_to_store:], to_store_labels[i].as_in_context(mx.cpu()), dim=0)
                else:
                    self.key_memory = nd.concat(self.key_memory, to_store_keys.as_in_context(mx.cpu()), dim=0)
                    for i in range(num_inputs):
                        self.value_memory[i] = nd.concat(self.value_memory[i], to_store_values[i].as_in_context(mx.cpu()), dim=0)
                    for i in range(num_outputs):
                        self.label_memory[i] = nd.concat(self.label_memory[i], to_store_labels[i].as_in_context(mx.cpu()), dim=0)

    def sample_memory(self, batch_size):
        num_stored_samples = self.key_memory.shape[0]
        if self.replay_batch_size == -1:
            sample_ind = nd.random.randint(0, num_stored_samples, (self.replay_steps, batch_size), ctx=mx.cpu())
        else:
            sample_ind = nd.random.randint(0, num_stored_samples, (self.replay_steps, self.replay_batch_size), ctx=mx.cpu())

        num_outputs = len(self.label_memory)

        sample_labels = [[self.label_memory[i][ind] for i in range(num_outputs)] for ind in sample_ind]
        sample_batches = [[[self.value_memory[j][ind] for j in range(len(self.value_memory))], sample_labels[i]] for i, ind in enumerate(sample_ind)]

        return sample_batches

    def sample_neighbours(self, data, query_network):
        num_stored_samples = self.key_memory.shape[0]
        batch_size = data[0].shape[0]

        query = query_network(*data).as_in_context(mx.cpu())

        vec1 = nd.repeat(query, repeats=num_stored_samples, axis=0)
        vec2 = nd.tile(self.key_memory, reps=(batch_size, 1))
        diff = nd.subtract(vec1, vec2)
        sq = nd.square(diff)
        batch_sum = nd.sum(sq, exclude=1, axis=0)
        sqrt = nd.sqrt(batch_sum)

        dist = nd.reshape(sqrt, shape=(batch_size, num_stored_samples))

        sample_ind = nd.topk(dist, k=self.k, axis=1, ret_typ="indices")
        num_outputs = len(self.label_memory)

        sample_labels = [self.label_memory[i][sample_ind] for i in range(num_outputs)]
        sample_batches = [[self.value_memory[j][sample_ind] for j in range(len(self.value_memory))], sample_labels]

        return sample_batches

    def get_query_network(self, context):
        lastEpoch = 0
        for file in os.listdir(self.query_net_dir):
            if self.query_net_prefix in file and ".json" in file:
                symbolFile = file

            if self.query_net_prefix in file and ".param" in file:
                epochStr = file.replace(".params", "").replace(self.query_net_prefix, "")
                epoch = int(epochStr)
                if epoch >= lastEpoch:
                    lastEpoch = epoch
                    weightFile = file

        inputNames = []
        if self.query_net_num_inputs == 1:
            inputNames.append("data")
        else:
            for i in range(self.query_net_num_inputs):
                inputNames.append("data" + str(i))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            net = mx.gluon.nn.SymbolBlock.imports(self.query_net_dir + symbolFile, inputNames, self.query_net_dir + weightFile, ctx=context[0])
        net.hybridize()
        return net
    
    def save_memory(self, path):
        mem_arr = [("keys", self.key_memory)] + [("values_"+str(k),v) for (k,v) in enumerate(self.value_memory)] + [("labels_"+str(k),v) for (k,v) in enumerate(self.label_memory)]
        mem_dict = {entry[0]:entry[1] for entry in mem_arr}
        nd.save(path, mem_dict)

    def load_memory(self, path):
        mem_dict = nd.load(path)
        self.value_memory = []
        self.label_memory = []
        for key in sorted(mem_dict.keys()):
            if key == "keys":
                self.key_memory = mem_dict[key]
            elif key.startswith("values_"):
                self.value_memory.append(mem_dict[key])
            elif key.startswith("labels_"):
                self.label_memory.append(mem_dict[key])


class Reparameterize(gluon.HybridBlock):
    def __init__(self, shape, pdf="normal", **kwargs):
        super(Reparameterize, self).__init__(**kwargs)
        self.sample_shape = shape
        self.latent_dim = shape[0]
        self.pdf = pdf

    def hybrid_forward(self, F, x):
        sample = None

        if self.pdf == "normal":
            eps = F.random_normal(shape=(-1,self.latent_dim))
            sample = x[0] + x[1] * eps

        return sample

class VectorQuantize(gluon.HybridBlock):
    def __init__(self, num_embeddings, embedding_dim, input_shape, total_feature_maps_size, **kwargs):
        super(VectorQuantize,self).__init__(**kwargs)
        self.embedding_dim = embedding_dim
        self.num_embeddings = num_embeddings

        # Codebook
        with self.name_scope():
            self.embeddings = self.params.get('embeddings', shape=(num_embeddings,embedding_dim), init=mx.init.Uniform())

    def hybrid_forward(self, F, x, *args, **params):
        # Change BCHW to BHWC
        x = F.swapaxes(x, 1, 2)
        x = F.swapaxes(x, 2, 3)

        #Shape: (BHW (all pixels), C (pixel vectors))
        flattened = F.reshape(x, shape=(-1, self.embedding_dim))

        # Get best representation from Codebook
        ab = F.dot(flattened, F.transpose(self.embeddings.var()))
        a2 = F.broadcast_like(F.sum(flattened ** 2, axis=1, keepdims=True),ab)
        b2 = F.broadcast_like(F.expand_dims(F.sum(self.embeddings.var() ** 2, axis=1),axis=0),ab)

        distances = a2 - 2*ab + b2
        encoding_indices = F.argmin(distances, axis=1)
        encodings = F.one_hot(encoding_indices, self.num_embeddings)

        # Quantize and unflatten
        quantized = F.reshape_like(F.dot(encodings, self.embeddings.var()), x)

        commit_l = F.mean((F.stop_gradient(quantized) - x) ** 2, axis=0, exclude=True)
        codebook_l = F.mean((quantized - F.stop_gradient(x)) ** 2, axis=0, exclude=True)

        quantized = x - F.stop_gradient(x + quantized)

        quantized = F.swapaxes(quantized, 3, 2)
        quantized = F.swapaxes(quantized, 2, 1)

        return [quantized, commit_l, codebook_l]

#Stream 0
class Net_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, batch_size=None, **kwargs):
        super(Net_0, self).__init__(**kwargs)
        with self.name_scope():
            self.save_specific_params_list = []
            if data_mean:
                assert(data_std)
                self.input_normalization_data_ = ZScoreNormalization(data_mean=data_mean['data_'],
                                                                               data_std=data_std['data_'])
            else:
                self.input_normalization_data_ = NoNormalization()

            self.conv1_padding = Padding(padding=(0,0,0,0,3,2,3,2))
            self.conv1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(7,7),
                strides=(2,2),
                use_bias=True)
            # conv1_, output shape: {[64,112,112]}

            self.batchnorm1_ = gluon.nn.BatchNorm()
            # batchnorm1_, output shape: {[64,112,112]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.pool1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.pool1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool1_, output shape: {[64,56,56]}

            self.conv3_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_1_, output shape: {[4,56,56]}

            self.batchnorm3_1_1_ = gluon.nn.BatchNorm()
            # batchnorm3_1_1_, output shape: {[4,56,56]}

            self.relu3_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_1_, output shape: {[4,56,56]}

            self.batchnorm4_1_1_ = gluon.nn.BatchNorm()
            # batchnorm4_1_1_, output shape: {[4,56,56]}

            self.relu4_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_1_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_1_, output shape: {[256,56,56]}

            self.batchnorm5_1_1_ = gluon.nn.BatchNorm()
            # batchnorm5_1_1_, output shape: {[256,56,56]}

            self.conv3_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_2_, output shape: {[4,56,56]}

            self.batchnorm3_1_2_ = gluon.nn.BatchNorm()
            # batchnorm3_1_2_, output shape: {[4,56,56]}

            self.relu3_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_2_, output shape: {[4,56,56]}

            self.batchnorm4_1_2_ = gluon.nn.BatchNorm()
            # batchnorm4_1_2_, output shape: {[4,56,56]}

            self.relu4_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_2_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_2_, output shape: {[256,56,56]}

            self.batchnorm5_1_2_ = gluon.nn.BatchNorm()
            # batchnorm5_1_2_, output shape: {[256,56,56]}

            self.conv3_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_3_, output shape: {[4,56,56]}

            self.batchnorm3_1_3_ = gluon.nn.BatchNorm()
            # batchnorm3_1_3_, output shape: {[4,56,56]}

            self.relu3_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_3_, output shape: {[4,56,56]}

            self.batchnorm4_1_3_ = gluon.nn.BatchNorm()
            # batchnorm4_1_3_, output shape: {[4,56,56]}

            self.relu4_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_3_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_3_, output shape: {[256,56,56]}

            self.batchnorm5_1_3_ = gluon.nn.BatchNorm()
            # batchnorm5_1_3_, output shape: {[256,56,56]}

            self.conv3_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_4_, output shape: {[4,56,56]}

            self.batchnorm3_1_4_ = gluon.nn.BatchNorm()
            # batchnorm3_1_4_, output shape: {[4,56,56]}

            self.relu3_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_4_, output shape: {[4,56,56]}

            self.batchnorm4_1_4_ = gluon.nn.BatchNorm()
            # batchnorm4_1_4_, output shape: {[4,56,56]}

            self.relu4_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_4_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_4_, output shape: {[256,56,56]}

            self.batchnorm5_1_4_ = gluon.nn.BatchNorm()
            # batchnorm5_1_4_, output shape: {[256,56,56]}

            self.conv3_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_5_, output shape: {[4,56,56]}

            self.batchnorm3_1_5_ = gluon.nn.BatchNorm()
            # batchnorm3_1_5_, output shape: {[4,56,56]}

            self.relu3_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_5_, output shape: {[4,56,56]}

            self.batchnorm4_1_5_ = gluon.nn.BatchNorm()
            # batchnorm4_1_5_, output shape: {[4,56,56]}

            self.relu4_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_5_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_5_, output shape: {[256,56,56]}

            self.batchnorm5_1_5_ = gluon.nn.BatchNorm()
            # batchnorm5_1_5_, output shape: {[256,56,56]}

            self.conv3_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_6_, output shape: {[4,56,56]}

            self.batchnorm3_1_6_ = gluon.nn.BatchNorm()
            # batchnorm3_1_6_, output shape: {[4,56,56]}

            self.relu3_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_6_, output shape: {[4,56,56]}

            self.batchnorm4_1_6_ = gluon.nn.BatchNorm()
            # batchnorm4_1_6_, output shape: {[4,56,56]}

            self.relu4_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_6_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_6_, output shape: {[256,56,56]}

            self.batchnorm5_1_6_ = gluon.nn.BatchNorm()
            # batchnorm5_1_6_, output shape: {[256,56,56]}

            self.conv3_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_7_, output shape: {[4,56,56]}

            self.batchnorm3_1_7_ = gluon.nn.BatchNorm()
            # batchnorm3_1_7_, output shape: {[4,56,56]}

            self.relu3_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_7_, output shape: {[4,56,56]}

            self.batchnorm4_1_7_ = gluon.nn.BatchNorm()
            # batchnorm4_1_7_, output shape: {[4,56,56]}

            self.relu4_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_7_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_7_, output shape: {[256,56,56]}

            self.batchnorm5_1_7_ = gluon.nn.BatchNorm()
            # batchnorm5_1_7_, output shape: {[256,56,56]}

            self.conv3_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_8_, output shape: {[4,56,56]}

            self.batchnorm3_1_8_ = gluon.nn.BatchNorm()
            # batchnorm3_1_8_, output shape: {[4,56,56]}

            self.relu3_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_8_, output shape: {[4,56,56]}

            self.batchnorm4_1_8_ = gluon.nn.BatchNorm()
            # batchnorm4_1_8_, output shape: {[4,56,56]}

            self.relu4_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_8_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_8_, output shape: {[256,56,56]}

            self.batchnorm5_1_8_ = gluon.nn.BatchNorm()
            # batchnorm5_1_8_, output shape: {[256,56,56]}

            self.conv3_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_9_, output shape: {[4,56,56]}

            self.batchnorm3_1_9_ = gluon.nn.BatchNorm()
            # batchnorm3_1_9_, output shape: {[4,56,56]}

            self.relu3_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_9_, output shape: {[4,56,56]}

            self.batchnorm4_1_9_ = gluon.nn.BatchNorm()
            # batchnorm4_1_9_, output shape: {[4,56,56]}

            self.relu4_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_9_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_9_, output shape: {[256,56,56]}

            self.batchnorm5_1_9_ = gluon.nn.BatchNorm()
            # batchnorm5_1_9_, output shape: {[256,56,56]}

            self.conv3_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_10_, output shape: {[4,56,56]}

            self.batchnorm3_1_10_ = gluon.nn.BatchNorm()
            # batchnorm3_1_10_, output shape: {[4,56,56]}

            self.relu3_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_10_, output shape: {[4,56,56]}

            self.batchnorm4_1_10_ = gluon.nn.BatchNorm()
            # batchnorm4_1_10_, output shape: {[4,56,56]}

            self.relu4_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_10_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_10_, output shape: {[256,56,56]}

            self.batchnorm5_1_10_ = gluon.nn.BatchNorm()
            # batchnorm5_1_10_, output shape: {[256,56,56]}

            self.conv3_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_11_, output shape: {[4,56,56]}

            self.batchnorm3_1_11_ = gluon.nn.BatchNorm()
            # batchnorm3_1_11_, output shape: {[4,56,56]}

            self.relu3_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_11_, output shape: {[4,56,56]}

            self.batchnorm4_1_11_ = gluon.nn.BatchNorm()
            # batchnorm4_1_11_, output shape: {[4,56,56]}

            self.relu4_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_11_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_11_, output shape: {[256,56,56]}

            self.batchnorm5_1_11_ = gluon.nn.BatchNorm()
            # batchnorm5_1_11_, output shape: {[256,56,56]}

            self.conv3_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_12_, output shape: {[4,56,56]}

            self.batchnorm3_1_12_ = gluon.nn.BatchNorm()
            # batchnorm3_1_12_, output shape: {[4,56,56]}

            self.relu3_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_12_, output shape: {[4,56,56]}

            self.batchnorm4_1_12_ = gluon.nn.BatchNorm()
            # batchnorm4_1_12_, output shape: {[4,56,56]}

            self.relu4_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_12_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_12_, output shape: {[256,56,56]}

            self.batchnorm5_1_12_ = gluon.nn.BatchNorm()
            # batchnorm5_1_12_, output shape: {[256,56,56]}

            self.conv3_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_13_, output shape: {[4,56,56]}

            self.batchnorm3_1_13_ = gluon.nn.BatchNorm()
            # batchnorm3_1_13_, output shape: {[4,56,56]}

            self.relu3_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_13_, output shape: {[4,56,56]}

            self.batchnorm4_1_13_ = gluon.nn.BatchNorm()
            # batchnorm4_1_13_, output shape: {[4,56,56]}

            self.relu4_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_13_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_13_, output shape: {[256,56,56]}

            self.batchnorm5_1_13_ = gluon.nn.BatchNorm()
            # batchnorm5_1_13_, output shape: {[256,56,56]}

            self.conv3_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_14_, output shape: {[4,56,56]}

            self.batchnorm3_1_14_ = gluon.nn.BatchNorm()
            # batchnorm3_1_14_, output shape: {[4,56,56]}

            self.relu3_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_14_, output shape: {[4,56,56]}

            self.batchnorm4_1_14_ = gluon.nn.BatchNorm()
            # batchnorm4_1_14_, output shape: {[4,56,56]}

            self.relu4_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_14_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_14_, output shape: {[256,56,56]}

            self.batchnorm5_1_14_ = gluon.nn.BatchNorm()
            # batchnorm5_1_14_, output shape: {[256,56,56]}

            self.conv3_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_15_, output shape: {[4,56,56]}

            self.batchnorm3_1_15_ = gluon.nn.BatchNorm()
            # batchnorm3_1_15_, output shape: {[4,56,56]}

            self.relu3_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_15_, output shape: {[4,56,56]}

            self.batchnorm4_1_15_ = gluon.nn.BatchNorm()
            # batchnorm4_1_15_, output shape: {[4,56,56]}

            self.relu4_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_15_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_15_, output shape: {[256,56,56]}

            self.batchnorm5_1_15_ = gluon.nn.BatchNorm()
            # batchnorm5_1_15_, output shape: {[256,56,56]}

            self.conv3_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_16_, output shape: {[4,56,56]}

            self.batchnorm3_1_16_ = gluon.nn.BatchNorm()
            # batchnorm3_1_16_, output shape: {[4,56,56]}

            self.relu3_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_16_, output shape: {[4,56,56]}

            self.batchnorm4_1_16_ = gluon.nn.BatchNorm()
            # batchnorm4_1_16_, output shape: {[4,56,56]}

            self.relu4_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_16_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_16_, output shape: {[256,56,56]}

            self.batchnorm5_1_16_ = gluon.nn.BatchNorm()
            # batchnorm5_1_16_, output shape: {[256,56,56]}

            self.conv3_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_17_, output shape: {[4,56,56]}

            self.batchnorm3_1_17_ = gluon.nn.BatchNorm()
            # batchnorm3_1_17_, output shape: {[4,56,56]}

            self.relu3_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_17_, output shape: {[4,56,56]}

            self.batchnorm4_1_17_ = gluon.nn.BatchNorm()
            # batchnorm4_1_17_, output shape: {[4,56,56]}

            self.relu4_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_17_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_17_, output shape: {[256,56,56]}

            self.batchnorm5_1_17_ = gluon.nn.BatchNorm()
            # batchnorm5_1_17_, output shape: {[256,56,56]}

            self.conv3_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_18_, output shape: {[4,56,56]}

            self.batchnorm3_1_18_ = gluon.nn.BatchNorm()
            # batchnorm3_1_18_, output shape: {[4,56,56]}

            self.relu3_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_18_, output shape: {[4,56,56]}

            self.batchnorm4_1_18_ = gluon.nn.BatchNorm()
            # batchnorm4_1_18_, output shape: {[4,56,56]}

            self.relu4_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_18_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_18_, output shape: {[256,56,56]}

            self.batchnorm5_1_18_ = gluon.nn.BatchNorm()
            # batchnorm5_1_18_, output shape: {[256,56,56]}

            self.conv3_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_19_, output shape: {[4,56,56]}

            self.batchnorm3_1_19_ = gluon.nn.BatchNorm()
            # batchnorm3_1_19_, output shape: {[4,56,56]}

            self.relu3_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_19_, output shape: {[4,56,56]}

            self.batchnorm4_1_19_ = gluon.nn.BatchNorm()
            # batchnorm4_1_19_, output shape: {[4,56,56]}

            self.relu4_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_19_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_19_, output shape: {[256,56,56]}

            self.batchnorm5_1_19_ = gluon.nn.BatchNorm()
            # batchnorm5_1_19_, output shape: {[256,56,56]}

            self.conv3_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_20_, output shape: {[4,56,56]}

            self.batchnorm3_1_20_ = gluon.nn.BatchNorm()
            # batchnorm3_1_20_, output shape: {[4,56,56]}

            self.relu3_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_20_, output shape: {[4,56,56]}

            self.batchnorm4_1_20_ = gluon.nn.BatchNorm()
            # batchnorm4_1_20_, output shape: {[4,56,56]}

            self.relu4_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_20_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_20_, output shape: {[256,56,56]}

            self.batchnorm5_1_20_ = gluon.nn.BatchNorm()
            # batchnorm5_1_20_, output shape: {[256,56,56]}

            self.conv3_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_21_, output shape: {[4,56,56]}

            self.batchnorm3_1_21_ = gluon.nn.BatchNorm()
            # batchnorm3_1_21_, output shape: {[4,56,56]}

            self.relu3_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_21_, output shape: {[4,56,56]}

            self.batchnorm4_1_21_ = gluon.nn.BatchNorm()
            # batchnorm4_1_21_, output shape: {[4,56,56]}

            self.relu4_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_21_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_21_, output shape: {[256,56,56]}

            self.batchnorm5_1_21_ = gluon.nn.BatchNorm()
            # batchnorm5_1_21_, output shape: {[256,56,56]}

            self.conv3_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_22_, output shape: {[4,56,56]}

            self.batchnorm3_1_22_ = gluon.nn.BatchNorm()
            # batchnorm3_1_22_, output shape: {[4,56,56]}

            self.relu3_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_22_, output shape: {[4,56,56]}

            self.batchnorm4_1_22_ = gluon.nn.BatchNorm()
            # batchnorm4_1_22_, output shape: {[4,56,56]}

            self.relu4_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_22_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_22_, output shape: {[256,56,56]}

            self.batchnorm5_1_22_ = gluon.nn.BatchNorm()
            # batchnorm5_1_22_, output shape: {[256,56,56]}

            self.conv3_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_23_, output shape: {[4,56,56]}

            self.batchnorm3_1_23_ = gluon.nn.BatchNorm()
            # batchnorm3_1_23_, output shape: {[4,56,56]}

            self.relu3_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_23_, output shape: {[4,56,56]}

            self.batchnorm4_1_23_ = gluon.nn.BatchNorm()
            # batchnorm4_1_23_, output shape: {[4,56,56]}

            self.relu4_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_23_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_23_, output shape: {[256,56,56]}

            self.batchnorm5_1_23_ = gluon.nn.BatchNorm()
            # batchnorm5_1_23_, output shape: {[256,56,56]}

            self.conv3_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_24_, output shape: {[4,56,56]}

            self.batchnorm3_1_24_ = gluon.nn.BatchNorm()
            # batchnorm3_1_24_, output shape: {[4,56,56]}

            self.relu3_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_24_, output shape: {[4,56,56]}

            self.batchnorm4_1_24_ = gluon.nn.BatchNorm()
            # batchnorm4_1_24_, output shape: {[4,56,56]}

            self.relu4_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_24_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_24_, output shape: {[256,56,56]}

            self.batchnorm5_1_24_ = gluon.nn.BatchNorm()
            # batchnorm5_1_24_, output shape: {[256,56,56]}

            self.conv3_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_25_, output shape: {[4,56,56]}

            self.batchnorm3_1_25_ = gluon.nn.BatchNorm()
            # batchnorm3_1_25_, output shape: {[4,56,56]}

            self.relu3_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_25_, output shape: {[4,56,56]}

            self.batchnorm4_1_25_ = gluon.nn.BatchNorm()
            # batchnorm4_1_25_, output shape: {[4,56,56]}

            self.relu4_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_25_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_25_, output shape: {[256,56,56]}

            self.batchnorm5_1_25_ = gluon.nn.BatchNorm()
            # batchnorm5_1_25_, output shape: {[256,56,56]}

            self.conv3_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_26_, output shape: {[4,56,56]}

            self.batchnorm3_1_26_ = gluon.nn.BatchNorm()
            # batchnorm3_1_26_, output shape: {[4,56,56]}

            self.relu3_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_26_, output shape: {[4,56,56]}

            self.batchnorm4_1_26_ = gluon.nn.BatchNorm()
            # batchnorm4_1_26_, output shape: {[4,56,56]}

            self.relu4_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_26_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_26_, output shape: {[256,56,56]}

            self.batchnorm5_1_26_ = gluon.nn.BatchNorm()
            # batchnorm5_1_26_, output shape: {[256,56,56]}

            self.conv3_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_27_, output shape: {[4,56,56]}

            self.batchnorm3_1_27_ = gluon.nn.BatchNorm()
            # batchnorm3_1_27_, output shape: {[4,56,56]}

            self.relu3_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_27_, output shape: {[4,56,56]}

            self.batchnorm4_1_27_ = gluon.nn.BatchNorm()
            # batchnorm4_1_27_, output shape: {[4,56,56]}

            self.relu4_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_27_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_27_, output shape: {[256,56,56]}

            self.batchnorm5_1_27_ = gluon.nn.BatchNorm()
            # batchnorm5_1_27_, output shape: {[256,56,56]}

            self.conv3_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_28_, output shape: {[4,56,56]}

            self.batchnorm3_1_28_ = gluon.nn.BatchNorm()
            # batchnorm3_1_28_, output shape: {[4,56,56]}

            self.relu3_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_28_, output shape: {[4,56,56]}

            self.batchnorm4_1_28_ = gluon.nn.BatchNorm()
            # batchnorm4_1_28_, output shape: {[4,56,56]}

            self.relu4_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_28_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_28_, output shape: {[256,56,56]}

            self.batchnorm5_1_28_ = gluon.nn.BatchNorm()
            # batchnorm5_1_28_, output shape: {[256,56,56]}

            self.conv3_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_29_, output shape: {[4,56,56]}

            self.batchnorm3_1_29_ = gluon.nn.BatchNorm()
            # batchnorm3_1_29_, output shape: {[4,56,56]}

            self.relu3_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_29_, output shape: {[4,56,56]}

            self.batchnorm4_1_29_ = gluon.nn.BatchNorm()
            # batchnorm4_1_29_, output shape: {[4,56,56]}

            self.relu4_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_29_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_29_, output shape: {[256,56,56]}

            self.batchnorm5_1_29_ = gluon.nn.BatchNorm()
            # batchnorm5_1_29_, output shape: {[256,56,56]}

            self.conv3_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_30_, output shape: {[4,56,56]}

            self.batchnorm3_1_30_ = gluon.nn.BatchNorm()
            # batchnorm3_1_30_, output shape: {[4,56,56]}

            self.relu3_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_30_, output shape: {[4,56,56]}

            self.batchnorm4_1_30_ = gluon.nn.BatchNorm()
            # batchnorm4_1_30_, output shape: {[4,56,56]}

            self.relu4_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_30_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_30_, output shape: {[256,56,56]}

            self.batchnorm5_1_30_ = gluon.nn.BatchNorm()
            # batchnorm5_1_30_, output shape: {[256,56,56]}

            self.conv3_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_31_, output shape: {[4,56,56]}

            self.batchnorm3_1_31_ = gluon.nn.BatchNorm()
            # batchnorm3_1_31_, output shape: {[4,56,56]}

            self.relu3_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_31_, output shape: {[4,56,56]}

            self.batchnorm4_1_31_ = gluon.nn.BatchNorm()
            # batchnorm4_1_31_, output shape: {[4,56,56]}

            self.relu4_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_31_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_31_, output shape: {[256,56,56]}

            self.batchnorm5_1_31_ = gluon.nn.BatchNorm()
            # batchnorm5_1_31_, output shape: {[256,56,56]}

            self.conv3_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv3_1_32_, output shape: {[4,56,56]}

            self.batchnorm3_1_32_ = gluon.nn.BatchNorm()
            # batchnorm3_1_32_, output shape: {[4,56,56]}

            self.relu3_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_32_, output shape: {[4,56,56]}

            self.batchnorm4_1_32_ = gluon.nn.BatchNorm()
            # batchnorm4_1_32_, output shape: {[4,56,56]}

            self.relu4_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_32_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv5_1_32_, output shape: {[256,56,56]}

            self.batchnorm5_1_32_ = gluon.nn.BatchNorm()
            # batchnorm5_1_32_, output shape: {[256,56,56]}

            self.conv2_2_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv2_2_, output shape: {[256,56,56]}

            self.batchnorm2_2_ = gluon.nn.BatchNorm()
            # batchnorm2_2_, output shape: {[256,56,56]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.conv9_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_1_, output shape: {[4,56,56]}

            self.batchnorm9_1_1_ = gluon.nn.BatchNorm()
            # batchnorm9_1_1_, output shape: {[4,56,56]}

            self.relu9_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_1_, output shape: {[4,56,56]}

            self.batchnorm10_1_1_ = gluon.nn.BatchNorm()
            # batchnorm10_1_1_, output shape: {[4,56,56]}

            self.relu10_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_1_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_1_, output shape: {[256,56,56]}

            self.batchnorm11_1_1_ = gluon.nn.BatchNorm()
            # batchnorm11_1_1_, output shape: {[256,56,56]}

            self.conv9_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_2_, output shape: {[4,56,56]}

            self.batchnorm9_1_2_ = gluon.nn.BatchNorm()
            # batchnorm9_1_2_, output shape: {[4,56,56]}

            self.relu9_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_2_, output shape: {[4,56,56]}

            self.batchnorm10_1_2_ = gluon.nn.BatchNorm()
            # batchnorm10_1_2_, output shape: {[4,56,56]}

            self.relu10_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_2_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_2_, output shape: {[256,56,56]}

            self.batchnorm11_1_2_ = gluon.nn.BatchNorm()
            # batchnorm11_1_2_, output shape: {[256,56,56]}

            self.conv9_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_3_, output shape: {[4,56,56]}

            self.batchnorm9_1_3_ = gluon.nn.BatchNorm()
            # batchnorm9_1_3_, output shape: {[4,56,56]}

            self.relu9_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_3_, output shape: {[4,56,56]}

            self.batchnorm10_1_3_ = gluon.nn.BatchNorm()
            # batchnorm10_1_3_, output shape: {[4,56,56]}

            self.relu10_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_3_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_3_, output shape: {[256,56,56]}

            self.batchnorm11_1_3_ = gluon.nn.BatchNorm()
            # batchnorm11_1_3_, output shape: {[256,56,56]}

            self.conv9_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_4_, output shape: {[4,56,56]}

            self.batchnorm9_1_4_ = gluon.nn.BatchNorm()
            # batchnorm9_1_4_, output shape: {[4,56,56]}

            self.relu9_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_4_, output shape: {[4,56,56]}

            self.batchnorm10_1_4_ = gluon.nn.BatchNorm()
            # batchnorm10_1_4_, output shape: {[4,56,56]}

            self.relu10_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_4_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_4_, output shape: {[256,56,56]}

            self.batchnorm11_1_4_ = gluon.nn.BatchNorm()
            # batchnorm11_1_4_, output shape: {[256,56,56]}

            self.conv9_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_5_, output shape: {[4,56,56]}

            self.batchnorm9_1_5_ = gluon.nn.BatchNorm()
            # batchnorm9_1_5_, output shape: {[4,56,56]}

            self.relu9_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_5_, output shape: {[4,56,56]}

            self.batchnorm10_1_5_ = gluon.nn.BatchNorm()
            # batchnorm10_1_5_, output shape: {[4,56,56]}

            self.relu10_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_5_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_5_, output shape: {[256,56,56]}

            self.batchnorm11_1_5_ = gluon.nn.BatchNorm()
            # batchnorm11_1_5_, output shape: {[256,56,56]}

            self.conv9_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_6_, output shape: {[4,56,56]}

            self.batchnorm9_1_6_ = gluon.nn.BatchNorm()
            # batchnorm9_1_6_, output shape: {[4,56,56]}

            self.relu9_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_6_, output shape: {[4,56,56]}

            self.batchnorm10_1_6_ = gluon.nn.BatchNorm()
            # batchnorm10_1_6_, output shape: {[4,56,56]}

            self.relu10_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_6_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_6_, output shape: {[256,56,56]}

            self.batchnorm11_1_6_ = gluon.nn.BatchNorm()
            # batchnorm11_1_6_, output shape: {[256,56,56]}

            self.conv9_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_7_, output shape: {[4,56,56]}

            self.batchnorm9_1_7_ = gluon.nn.BatchNorm()
            # batchnorm9_1_7_, output shape: {[4,56,56]}

            self.relu9_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_7_, output shape: {[4,56,56]}

            self.batchnorm10_1_7_ = gluon.nn.BatchNorm()
            # batchnorm10_1_7_, output shape: {[4,56,56]}

            self.relu10_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_7_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_7_, output shape: {[256,56,56]}

            self.batchnorm11_1_7_ = gluon.nn.BatchNorm()
            # batchnorm11_1_7_, output shape: {[256,56,56]}

            self.conv9_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_8_, output shape: {[4,56,56]}

            self.batchnorm9_1_8_ = gluon.nn.BatchNorm()
            # batchnorm9_1_8_, output shape: {[4,56,56]}

            self.relu9_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_8_, output shape: {[4,56,56]}

            self.batchnorm10_1_8_ = gluon.nn.BatchNorm()
            # batchnorm10_1_8_, output shape: {[4,56,56]}

            self.relu10_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_8_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_8_, output shape: {[256,56,56]}

            self.batchnorm11_1_8_ = gluon.nn.BatchNorm()
            # batchnorm11_1_8_, output shape: {[256,56,56]}

            self.conv9_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_9_, output shape: {[4,56,56]}

            self.batchnorm9_1_9_ = gluon.nn.BatchNorm()
            # batchnorm9_1_9_, output shape: {[4,56,56]}

            self.relu9_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_9_, output shape: {[4,56,56]}

            self.batchnorm10_1_9_ = gluon.nn.BatchNorm()
            # batchnorm10_1_9_, output shape: {[4,56,56]}

            self.relu10_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_9_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_9_, output shape: {[256,56,56]}

            self.batchnorm11_1_9_ = gluon.nn.BatchNorm()
            # batchnorm11_1_9_, output shape: {[256,56,56]}

            self.conv9_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_10_, output shape: {[4,56,56]}

            self.batchnorm9_1_10_ = gluon.nn.BatchNorm()
            # batchnorm9_1_10_, output shape: {[4,56,56]}

            self.relu9_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_10_, output shape: {[4,56,56]}

            self.batchnorm10_1_10_ = gluon.nn.BatchNorm()
            # batchnorm10_1_10_, output shape: {[4,56,56]}

            self.relu10_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_10_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_10_, output shape: {[256,56,56]}

            self.batchnorm11_1_10_ = gluon.nn.BatchNorm()
            # batchnorm11_1_10_, output shape: {[256,56,56]}

            self.conv9_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_11_, output shape: {[4,56,56]}

            self.batchnorm9_1_11_ = gluon.nn.BatchNorm()
            # batchnorm9_1_11_, output shape: {[4,56,56]}

            self.relu9_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_11_, output shape: {[4,56,56]}

            self.batchnorm10_1_11_ = gluon.nn.BatchNorm()
            # batchnorm10_1_11_, output shape: {[4,56,56]}

            self.relu10_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_11_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_11_, output shape: {[256,56,56]}

            self.batchnorm11_1_11_ = gluon.nn.BatchNorm()
            # batchnorm11_1_11_, output shape: {[256,56,56]}

            self.conv9_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_12_, output shape: {[4,56,56]}

            self.batchnorm9_1_12_ = gluon.nn.BatchNorm()
            # batchnorm9_1_12_, output shape: {[4,56,56]}

            self.relu9_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_12_, output shape: {[4,56,56]}

            self.batchnorm10_1_12_ = gluon.nn.BatchNorm()
            # batchnorm10_1_12_, output shape: {[4,56,56]}

            self.relu10_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_12_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_12_, output shape: {[256,56,56]}

            self.batchnorm11_1_12_ = gluon.nn.BatchNorm()
            # batchnorm11_1_12_, output shape: {[256,56,56]}

            self.conv9_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_13_, output shape: {[4,56,56]}

            self.batchnorm9_1_13_ = gluon.nn.BatchNorm()
            # batchnorm9_1_13_, output shape: {[4,56,56]}

            self.relu9_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_13_, output shape: {[4,56,56]}

            self.batchnorm10_1_13_ = gluon.nn.BatchNorm()
            # batchnorm10_1_13_, output shape: {[4,56,56]}

            self.relu10_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_13_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_13_, output shape: {[256,56,56]}

            self.batchnorm11_1_13_ = gluon.nn.BatchNorm()
            # batchnorm11_1_13_, output shape: {[256,56,56]}

            self.conv9_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_14_, output shape: {[4,56,56]}

            self.batchnorm9_1_14_ = gluon.nn.BatchNorm()
            # batchnorm9_1_14_, output shape: {[4,56,56]}

            self.relu9_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_14_, output shape: {[4,56,56]}

            self.batchnorm10_1_14_ = gluon.nn.BatchNorm()
            # batchnorm10_1_14_, output shape: {[4,56,56]}

            self.relu10_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_14_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_14_, output shape: {[256,56,56]}

            self.batchnorm11_1_14_ = gluon.nn.BatchNorm()
            # batchnorm11_1_14_, output shape: {[256,56,56]}

            self.conv9_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_15_, output shape: {[4,56,56]}

            self.batchnorm9_1_15_ = gluon.nn.BatchNorm()
            # batchnorm9_1_15_, output shape: {[4,56,56]}

            self.relu9_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_15_, output shape: {[4,56,56]}

            self.batchnorm10_1_15_ = gluon.nn.BatchNorm()
            # batchnorm10_1_15_, output shape: {[4,56,56]}

            self.relu10_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_15_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_15_, output shape: {[256,56,56]}

            self.batchnorm11_1_15_ = gluon.nn.BatchNorm()
            # batchnorm11_1_15_, output shape: {[256,56,56]}

            self.conv9_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_16_, output shape: {[4,56,56]}

            self.batchnorm9_1_16_ = gluon.nn.BatchNorm()
            # batchnorm9_1_16_, output shape: {[4,56,56]}

            self.relu9_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_16_, output shape: {[4,56,56]}

            self.batchnorm10_1_16_ = gluon.nn.BatchNorm()
            # batchnorm10_1_16_, output shape: {[4,56,56]}

            self.relu10_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_16_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_16_, output shape: {[256,56,56]}

            self.batchnorm11_1_16_ = gluon.nn.BatchNorm()
            # batchnorm11_1_16_, output shape: {[256,56,56]}

            self.conv9_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_17_, output shape: {[4,56,56]}

            self.batchnorm9_1_17_ = gluon.nn.BatchNorm()
            # batchnorm9_1_17_, output shape: {[4,56,56]}

            self.relu9_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_17_, output shape: {[4,56,56]}

            self.batchnorm10_1_17_ = gluon.nn.BatchNorm()
            # batchnorm10_1_17_, output shape: {[4,56,56]}

            self.relu10_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_17_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_17_, output shape: {[256,56,56]}

            self.batchnorm11_1_17_ = gluon.nn.BatchNorm()
            # batchnorm11_1_17_, output shape: {[256,56,56]}

            self.conv9_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_18_, output shape: {[4,56,56]}

            self.batchnorm9_1_18_ = gluon.nn.BatchNorm()
            # batchnorm9_1_18_, output shape: {[4,56,56]}

            self.relu9_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_18_, output shape: {[4,56,56]}

            self.batchnorm10_1_18_ = gluon.nn.BatchNorm()
            # batchnorm10_1_18_, output shape: {[4,56,56]}

            self.relu10_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_18_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_18_, output shape: {[256,56,56]}

            self.batchnorm11_1_18_ = gluon.nn.BatchNorm()
            # batchnorm11_1_18_, output shape: {[256,56,56]}

            self.conv9_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_19_, output shape: {[4,56,56]}

            self.batchnorm9_1_19_ = gluon.nn.BatchNorm()
            # batchnorm9_1_19_, output shape: {[4,56,56]}

            self.relu9_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_19_, output shape: {[4,56,56]}

            self.batchnorm10_1_19_ = gluon.nn.BatchNorm()
            # batchnorm10_1_19_, output shape: {[4,56,56]}

            self.relu10_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_19_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_19_, output shape: {[256,56,56]}

            self.batchnorm11_1_19_ = gluon.nn.BatchNorm()
            # batchnorm11_1_19_, output shape: {[256,56,56]}

            self.conv9_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_20_, output shape: {[4,56,56]}

            self.batchnorm9_1_20_ = gluon.nn.BatchNorm()
            # batchnorm9_1_20_, output shape: {[4,56,56]}

            self.relu9_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_20_, output shape: {[4,56,56]}

            self.batchnorm10_1_20_ = gluon.nn.BatchNorm()
            # batchnorm10_1_20_, output shape: {[4,56,56]}

            self.relu10_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_20_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_20_, output shape: {[256,56,56]}

            self.batchnorm11_1_20_ = gluon.nn.BatchNorm()
            # batchnorm11_1_20_, output shape: {[256,56,56]}

            self.conv9_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_21_, output shape: {[4,56,56]}

            self.batchnorm9_1_21_ = gluon.nn.BatchNorm()
            # batchnorm9_1_21_, output shape: {[4,56,56]}

            self.relu9_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_21_, output shape: {[4,56,56]}

            self.batchnorm10_1_21_ = gluon.nn.BatchNorm()
            # batchnorm10_1_21_, output shape: {[4,56,56]}

            self.relu10_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_21_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_21_, output shape: {[256,56,56]}

            self.batchnorm11_1_21_ = gluon.nn.BatchNorm()
            # batchnorm11_1_21_, output shape: {[256,56,56]}

            self.conv9_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_22_, output shape: {[4,56,56]}

            self.batchnorm9_1_22_ = gluon.nn.BatchNorm()
            # batchnorm9_1_22_, output shape: {[4,56,56]}

            self.relu9_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_22_, output shape: {[4,56,56]}

            self.batchnorm10_1_22_ = gluon.nn.BatchNorm()
            # batchnorm10_1_22_, output shape: {[4,56,56]}

            self.relu10_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_22_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_22_, output shape: {[256,56,56]}

            self.batchnorm11_1_22_ = gluon.nn.BatchNorm()
            # batchnorm11_1_22_, output shape: {[256,56,56]}

            self.conv9_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_23_, output shape: {[4,56,56]}

            self.batchnorm9_1_23_ = gluon.nn.BatchNorm()
            # batchnorm9_1_23_, output shape: {[4,56,56]}

            self.relu9_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_23_, output shape: {[4,56,56]}

            self.batchnorm10_1_23_ = gluon.nn.BatchNorm()
            # batchnorm10_1_23_, output shape: {[4,56,56]}

            self.relu10_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_23_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_23_, output shape: {[256,56,56]}

            self.batchnorm11_1_23_ = gluon.nn.BatchNorm()
            # batchnorm11_1_23_, output shape: {[256,56,56]}

            self.conv9_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_24_, output shape: {[4,56,56]}

            self.batchnorm9_1_24_ = gluon.nn.BatchNorm()
            # batchnorm9_1_24_, output shape: {[4,56,56]}

            self.relu9_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_24_, output shape: {[4,56,56]}

            self.batchnorm10_1_24_ = gluon.nn.BatchNorm()
            # batchnorm10_1_24_, output shape: {[4,56,56]}

            self.relu10_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_24_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_24_, output shape: {[256,56,56]}

            self.batchnorm11_1_24_ = gluon.nn.BatchNorm()
            # batchnorm11_1_24_, output shape: {[256,56,56]}

            self.conv9_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_25_, output shape: {[4,56,56]}

            self.batchnorm9_1_25_ = gluon.nn.BatchNorm()
            # batchnorm9_1_25_, output shape: {[4,56,56]}

            self.relu9_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_25_, output shape: {[4,56,56]}

            self.batchnorm10_1_25_ = gluon.nn.BatchNorm()
            # batchnorm10_1_25_, output shape: {[4,56,56]}

            self.relu10_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_25_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_25_, output shape: {[256,56,56]}

            self.batchnorm11_1_25_ = gluon.nn.BatchNorm()
            # batchnorm11_1_25_, output shape: {[256,56,56]}

            self.conv9_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_26_, output shape: {[4,56,56]}

            self.batchnorm9_1_26_ = gluon.nn.BatchNorm()
            # batchnorm9_1_26_, output shape: {[4,56,56]}

            self.relu9_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_26_, output shape: {[4,56,56]}

            self.batchnorm10_1_26_ = gluon.nn.BatchNorm()
            # batchnorm10_1_26_, output shape: {[4,56,56]}

            self.relu10_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_26_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_26_, output shape: {[256,56,56]}

            self.batchnorm11_1_26_ = gluon.nn.BatchNorm()
            # batchnorm11_1_26_, output shape: {[256,56,56]}

            self.conv9_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_27_, output shape: {[4,56,56]}

            self.batchnorm9_1_27_ = gluon.nn.BatchNorm()
            # batchnorm9_1_27_, output shape: {[4,56,56]}

            self.relu9_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_27_, output shape: {[4,56,56]}

            self.batchnorm10_1_27_ = gluon.nn.BatchNorm()
            # batchnorm10_1_27_, output shape: {[4,56,56]}

            self.relu10_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_27_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_27_, output shape: {[256,56,56]}

            self.batchnorm11_1_27_ = gluon.nn.BatchNorm()
            # batchnorm11_1_27_, output shape: {[256,56,56]}

            self.conv9_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_28_, output shape: {[4,56,56]}

            self.batchnorm9_1_28_ = gluon.nn.BatchNorm()
            # batchnorm9_1_28_, output shape: {[4,56,56]}

            self.relu9_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_28_, output shape: {[4,56,56]}

            self.batchnorm10_1_28_ = gluon.nn.BatchNorm()
            # batchnorm10_1_28_, output shape: {[4,56,56]}

            self.relu10_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_28_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_28_, output shape: {[256,56,56]}

            self.batchnorm11_1_28_ = gluon.nn.BatchNorm()
            # batchnorm11_1_28_, output shape: {[256,56,56]}

            self.conv9_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_29_, output shape: {[4,56,56]}

            self.batchnorm9_1_29_ = gluon.nn.BatchNorm()
            # batchnorm9_1_29_, output shape: {[4,56,56]}

            self.relu9_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_29_, output shape: {[4,56,56]}

            self.batchnorm10_1_29_ = gluon.nn.BatchNorm()
            # batchnorm10_1_29_, output shape: {[4,56,56]}

            self.relu10_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_29_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_29_, output shape: {[256,56,56]}

            self.batchnorm11_1_29_ = gluon.nn.BatchNorm()
            # batchnorm11_1_29_, output shape: {[256,56,56]}

            self.conv9_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_30_, output shape: {[4,56,56]}

            self.batchnorm9_1_30_ = gluon.nn.BatchNorm()
            # batchnorm9_1_30_, output shape: {[4,56,56]}

            self.relu9_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_30_, output shape: {[4,56,56]}

            self.batchnorm10_1_30_ = gluon.nn.BatchNorm()
            # batchnorm10_1_30_, output shape: {[4,56,56]}

            self.relu10_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_30_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_30_, output shape: {[256,56,56]}

            self.batchnorm11_1_30_ = gluon.nn.BatchNorm()
            # batchnorm11_1_30_, output shape: {[256,56,56]}

            self.conv9_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_31_, output shape: {[4,56,56]}

            self.batchnorm9_1_31_ = gluon.nn.BatchNorm()
            # batchnorm9_1_31_, output shape: {[4,56,56]}

            self.relu9_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_31_, output shape: {[4,56,56]}

            self.batchnorm10_1_31_ = gluon.nn.BatchNorm()
            # batchnorm10_1_31_, output shape: {[4,56,56]}

            self.relu10_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_31_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_31_, output shape: {[256,56,56]}

            self.batchnorm11_1_31_ = gluon.nn.BatchNorm()
            # batchnorm11_1_31_, output shape: {[256,56,56]}

            self.conv9_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv9_1_32_, output shape: {[4,56,56]}

            self.batchnorm9_1_32_ = gluon.nn.BatchNorm()
            # batchnorm9_1_32_, output shape: {[4,56,56]}

            self.relu9_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv10_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_1_32_, output shape: {[4,56,56]}

            self.batchnorm10_1_32_ = gluon.nn.BatchNorm()
            # batchnorm10_1_32_, output shape: {[4,56,56]}

            self.relu10_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_32_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv11_1_32_, output shape: {[256,56,56]}

            self.batchnorm11_1_32_ = gluon.nn.BatchNorm()
            # batchnorm11_1_32_, output shape: {[256,56,56]}

            self.relu13_ = gluon.nn.Activation(activation='relu')
            self.conv15_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_1_, output shape: {[4,56,56]}

            self.batchnorm15_1_1_ = gluon.nn.BatchNorm()
            # batchnorm15_1_1_, output shape: {[4,56,56]}

            self.relu15_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_1_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_1_, output shape: {[4,56,56]}

            self.batchnorm16_1_1_ = gluon.nn.BatchNorm()
            # batchnorm16_1_1_, output shape: {[4,56,56]}

            self.relu16_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_1_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_1_, output shape: {[256,56,56]}

            self.batchnorm17_1_1_ = gluon.nn.BatchNorm()
            # batchnorm17_1_1_, output shape: {[256,56,56]}

            self.conv15_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_2_, output shape: {[4,56,56]}

            self.batchnorm15_1_2_ = gluon.nn.BatchNorm()
            # batchnorm15_1_2_, output shape: {[4,56,56]}

            self.relu15_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_2_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_2_, output shape: {[4,56,56]}

            self.batchnorm16_1_2_ = gluon.nn.BatchNorm()
            # batchnorm16_1_2_, output shape: {[4,56,56]}

            self.relu16_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_2_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_2_, output shape: {[256,56,56]}

            self.batchnorm17_1_2_ = gluon.nn.BatchNorm()
            # batchnorm17_1_2_, output shape: {[256,56,56]}

            self.conv15_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_3_, output shape: {[4,56,56]}

            self.batchnorm15_1_3_ = gluon.nn.BatchNorm()
            # batchnorm15_1_3_, output shape: {[4,56,56]}

            self.relu15_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_3_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_3_, output shape: {[4,56,56]}

            self.batchnorm16_1_3_ = gluon.nn.BatchNorm()
            # batchnorm16_1_3_, output shape: {[4,56,56]}

            self.relu16_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_3_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_3_, output shape: {[256,56,56]}

            self.batchnorm17_1_3_ = gluon.nn.BatchNorm()
            # batchnorm17_1_3_, output shape: {[256,56,56]}

            self.conv15_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_4_, output shape: {[4,56,56]}

            self.batchnorm15_1_4_ = gluon.nn.BatchNorm()
            # batchnorm15_1_4_, output shape: {[4,56,56]}

            self.relu15_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_4_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_4_, output shape: {[4,56,56]}

            self.batchnorm16_1_4_ = gluon.nn.BatchNorm()
            # batchnorm16_1_4_, output shape: {[4,56,56]}

            self.relu16_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_4_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_4_, output shape: {[256,56,56]}

            self.batchnorm17_1_4_ = gluon.nn.BatchNorm()
            # batchnorm17_1_4_, output shape: {[256,56,56]}

            self.conv15_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_5_, output shape: {[4,56,56]}

            self.batchnorm15_1_5_ = gluon.nn.BatchNorm()
            # batchnorm15_1_5_, output shape: {[4,56,56]}

            self.relu15_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_5_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_5_, output shape: {[4,56,56]}

            self.batchnorm16_1_5_ = gluon.nn.BatchNorm()
            # batchnorm16_1_5_, output shape: {[4,56,56]}

            self.relu16_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_5_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_5_, output shape: {[256,56,56]}

            self.batchnorm17_1_5_ = gluon.nn.BatchNorm()
            # batchnorm17_1_5_, output shape: {[256,56,56]}

            self.conv15_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_6_, output shape: {[4,56,56]}

            self.batchnorm15_1_6_ = gluon.nn.BatchNorm()
            # batchnorm15_1_6_, output shape: {[4,56,56]}

            self.relu15_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_6_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_6_, output shape: {[4,56,56]}

            self.batchnorm16_1_6_ = gluon.nn.BatchNorm()
            # batchnorm16_1_6_, output shape: {[4,56,56]}

            self.relu16_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_6_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_6_, output shape: {[256,56,56]}

            self.batchnorm17_1_6_ = gluon.nn.BatchNorm()
            # batchnorm17_1_6_, output shape: {[256,56,56]}

            self.conv15_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_7_, output shape: {[4,56,56]}

            self.batchnorm15_1_7_ = gluon.nn.BatchNorm()
            # batchnorm15_1_7_, output shape: {[4,56,56]}

            self.relu15_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_7_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_7_, output shape: {[4,56,56]}

            self.batchnorm16_1_7_ = gluon.nn.BatchNorm()
            # batchnorm16_1_7_, output shape: {[4,56,56]}

            self.relu16_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_7_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_7_, output shape: {[256,56,56]}

            self.batchnorm17_1_7_ = gluon.nn.BatchNorm()
            # batchnorm17_1_7_, output shape: {[256,56,56]}

            self.conv15_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_8_, output shape: {[4,56,56]}

            self.batchnorm15_1_8_ = gluon.nn.BatchNorm()
            # batchnorm15_1_8_, output shape: {[4,56,56]}

            self.relu15_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_8_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_8_, output shape: {[4,56,56]}

            self.batchnorm16_1_8_ = gluon.nn.BatchNorm()
            # batchnorm16_1_8_, output shape: {[4,56,56]}

            self.relu16_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_8_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_8_, output shape: {[256,56,56]}

            self.batchnorm17_1_8_ = gluon.nn.BatchNorm()
            # batchnorm17_1_8_, output shape: {[256,56,56]}

            self.conv15_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_9_, output shape: {[4,56,56]}

            self.batchnorm15_1_9_ = gluon.nn.BatchNorm()
            # batchnorm15_1_9_, output shape: {[4,56,56]}

            self.relu15_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_9_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_9_, output shape: {[4,56,56]}

            self.batchnorm16_1_9_ = gluon.nn.BatchNorm()
            # batchnorm16_1_9_, output shape: {[4,56,56]}

            self.relu16_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_9_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_9_, output shape: {[256,56,56]}

            self.batchnorm17_1_9_ = gluon.nn.BatchNorm()
            # batchnorm17_1_9_, output shape: {[256,56,56]}

            self.conv15_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_10_, output shape: {[4,56,56]}

            self.batchnorm15_1_10_ = gluon.nn.BatchNorm()
            # batchnorm15_1_10_, output shape: {[4,56,56]}

            self.relu15_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_10_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_10_, output shape: {[4,56,56]}

            self.batchnorm16_1_10_ = gluon.nn.BatchNorm()
            # batchnorm16_1_10_, output shape: {[4,56,56]}

            self.relu16_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_10_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_10_, output shape: {[256,56,56]}

            self.batchnorm17_1_10_ = gluon.nn.BatchNorm()
            # batchnorm17_1_10_, output shape: {[256,56,56]}

            self.conv15_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_11_, output shape: {[4,56,56]}

            self.batchnorm15_1_11_ = gluon.nn.BatchNorm()
            # batchnorm15_1_11_, output shape: {[4,56,56]}

            self.relu15_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_11_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_11_, output shape: {[4,56,56]}

            self.batchnorm16_1_11_ = gluon.nn.BatchNorm()
            # batchnorm16_1_11_, output shape: {[4,56,56]}

            self.relu16_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_11_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_11_, output shape: {[256,56,56]}

            self.batchnorm17_1_11_ = gluon.nn.BatchNorm()
            # batchnorm17_1_11_, output shape: {[256,56,56]}

            self.conv15_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_12_, output shape: {[4,56,56]}

            self.batchnorm15_1_12_ = gluon.nn.BatchNorm()
            # batchnorm15_1_12_, output shape: {[4,56,56]}

            self.relu15_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_12_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_12_, output shape: {[4,56,56]}

            self.batchnorm16_1_12_ = gluon.nn.BatchNorm()
            # batchnorm16_1_12_, output shape: {[4,56,56]}

            self.relu16_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_12_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_12_, output shape: {[256,56,56]}

            self.batchnorm17_1_12_ = gluon.nn.BatchNorm()
            # batchnorm17_1_12_, output shape: {[256,56,56]}

            self.conv15_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_13_, output shape: {[4,56,56]}

            self.batchnorm15_1_13_ = gluon.nn.BatchNorm()
            # batchnorm15_1_13_, output shape: {[4,56,56]}

            self.relu15_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_13_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_13_, output shape: {[4,56,56]}

            self.batchnorm16_1_13_ = gluon.nn.BatchNorm()
            # batchnorm16_1_13_, output shape: {[4,56,56]}

            self.relu16_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_13_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_13_, output shape: {[256,56,56]}

            self.batchnorm17_1_13_ = gluon.nn.BatchNorm()
            # batchnorm17_1_13_, output shape: {[256,56,56]}

            self.conv15_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_14_, output shape: {[4,56,56]}

            self.batchnorm15_1_14_ = gluon.nn.BatchNorm()
            # batchnorm15_1_14_, output shape: {[4,56,56]}

            self.relu15_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_14_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_14_, output shape: {[4,56,56]}

            self.batchnorm16_1_14_ = gluon.nn.BatchNorm()
            # batchnorm16_1_14_, output shape: {[4,56,56]}

            self.relu16_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_14_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_14_, output shape: {[256,56,56]}

            self.batchnorm17_1_14_ = gluon.nn.BatchNorm()
            # batchnorm17_1_14_, output shape: {[256,56,56]}

            self.conv15_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_15_, output shape: {[4,56,56]}

            self.batchnorm15_1_15_ = gluon.nn.BatchNorm()
            # batchnorm15_1_15_, output shape: {[4,56,56]}

            self.relu15_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_15_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_15_, output shape: {[4,56,56]}

            self.batchnorm16_1_15_ = gluon.nn.BatchNorm()
            # batchnorm16_1_15_, output shape: {[4,56,56]}

            self.relu16_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_15_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_15_, output shape: {[256,56,56]}

            self.batchnorm17_1_15_ = gluon.nn.BatchNorm()
            # batchnorm17_1_15_, output shape: {[256,56,56]}

            self.conv15_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_16_, output shape: {[4,56,56]}

            self.batchnorm15_1_16_ = gluon.nn.BatchNorm()
            # batchnorm15_1_16_, output shape: {[4,56,56]}

            self.relu15_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_16_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_16_, output shape: {[4,56,56]}

            self.batchnorm16_1_16_ = gluon.nn.BatchNorm()
            # batchnorm16_1_16_, output shape: {[4,56,56]}

            self.relu16_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_16_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_16_, output shape: {[256,56,56]}

            self.batchnorm17_1_16_ = gluon.nn.BatchNorm()
            # batchnorm17_1_16_, output shape: {[256,56,56]}

            self.conv15_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_17_, output shape: {[4,56,56]}

            self.batchnorm15_1_17_ = gluon.nn.BatchNorm()
            # batchnorm15_1_17_, output shape: {[4,56,56]}

            self.relu15_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_17_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_17_, output shape: {[4,56,56]}

            self.batchnorm16_1_17_ = gluon.nn.BatchNorm()
            # batchnorm16_1_17_, output shape: {[4,56,56]}

            self.relu16_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_17_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_17_, output shape: {[256,56,56]}

            self.batchnorm17_1_17_ = gluon.nn.BatchNorm()
            # batchnorm17_1_17_, output shape: {[256,56,56]}

            self.conv15_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_18_, output shape: {[4,56,56]}

            self.batchnorm15_1_18_ = gluon.nn.BatchNorm()
            # batchnorm15_1_18_, output shape: {[4,56,56]}

            self.relu15_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_18_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_18_, output shape: {[4,56,56]}

            self.batchnorm16_1_18_ = gluon.nn.BatchNorm()
            # batchnorm16_1_18_, output shape: {[4,56,56]}

            self.relu16_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_18_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_18_, output shape: {[256,56,56]}

            self.batchnorm17_1_18_ = gluon.nn.BatchNorm()
            # batchnorm17_1_18_, output shape: {[256,56,56]}

            self.conv15_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_19_, output shape: {[4,56,56]}

            self.batchnorm15_1_19_ = gluon.nn.BatchNorm()
            # batchnorm15_1_19_, output shape: {[4,56,56]}

            self.relu15_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_19_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_19_, output shape: {[4,56,56]}

            self.batchnorm16_1_19_ = gluon.nn.BatchNorm()
            # batchnorm16_1_19_, output shape: {[4,56,56]}

            self.relu16_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_19_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_19_, output shape: {[256,56,56]}

            self.batchnorm17_1_19_ = gluon.nn.BatchNorm()
            # batchnorm17_1_19_, output shape: {[256,56,56]}

            self.conv15_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_20_, output shape: {[4,56,56]}

            self.batchnorm15_1_20_ = gluon.nn.BatchNorm()
            # batchnorm15_1_20_, output shape: {[4,56,56]}

            self.relu15_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_20_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_20_, output shape: {[4,56,56]}

            self.batchnorm16_1_20_ = gluon.nn.BatchNorm()
            # batchnorm16_1_20_, output shape: {[4,56,56]}

            self.relu16_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_20_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_20_, output shape: {[256,56,56]}

            self.batchnorm17_1_20_ = gluon.nn.BatchNorm()
            # batchnorm17_1_20_, output shape: {[256,56,56]}

            self.conv15_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_21_, output shape: {[4,56,56]}

            self.batchnorm15_1_21_ = gluon.nn.BatchNorm()
            # batchnorm15_1_21_, output shape: {[4,56,56]}

            self.relu15_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_21_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_21_, output shape: {[4,56,56]}

            self.batchnorm16_1_21_ = gluon.nn.BatchNorm()
            # batchnorm16_1_21_, output shape: {[4,56,56]}

            self.relu16_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_21_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_21_, output shape: {[256,56,56]}

            self.batchnorm17_1_21_ = gluon.nn.BatchNorm()
            # batchnorm17_1_21_, output shape: {[256,56,56]}

            self.conv15_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_22_, output shape: {[4,56,56]}

            self.batchnorm15_1_22_ = gluon.nn.BatchNorm()
            # batchnorm15_1_22_, output shape: {[4,56,56]}

            self.relu15_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_22_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_22_, output shape: {[4,56,56]}

            self.batchnorm16_1_22_ = gluon.nn.BatchNorm()
            # batchnorm16_1_22_, output shape: {[4,56,56]}

            self.relu16_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_22_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_22_, output shape: {[256,56,56]}

            self.batchnorm17_1_22_ = gluon.nn.BatchNorm()
            # batchnorm17_1_22_, output shape: {[256,56,56]}

            self.conv15_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_23_, output shape: {[4,56,56]}

            self.batchnorm15_1_23_ = gluon.nn.BatchNorm()
            # batchnorm15_1_23_, output shape: {[4,56,56]}

            self.relu15_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_23_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_23_, output shape: {[4,56,56]}

            self.batchnorm16_1_23_ = gluon.nn.BatchNorm()
            # batchnorm16_1_23_, output shape: {[4,56,56]}

            self.relu16_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_23_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_23_, output shape: {[256,56,56]}

            self.batchnorm17_1_23_ = gluon.nn.BatchNorm()
            # batchnorm17_1_23_, output shape: {[256,56,56]}

            self.conv15_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_24_, output shape: {[4,56,56]}

            self.batchnorm15_1_24_ = gluon.nn.BatchNorm()
            # batchnorm15_1_24_, output shape: {[4,56,56]}

            self.relu15_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_24_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_24_, output shape: {[4,56,56]}

            self.batchnorm16_1_24_ = gluon.nn.BatchNorm()
            # batchnorm16_1_24_, output shape: {[4,56,56]}

            self.relu16_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_24_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_24_, output shape: {[256,56,56]}

            self.batchnorm17_1_24_ = gluon.nn.BatchNorm()
            # batchnorm17_1_24_, output shape: {[256,56,56]}

            self.conv15_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_25_, output shape: {[4,56,56]}

            self.batchnorm15_1_25_ = gluon.nn.BatchNorm()
            # batchnorm15_1_25_, output shape: {[4,56,56]}

            self.relu15_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_25_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_25_, output shape: {[4,56,56]}

            self.batchnorm16_1_25_ = gluon.nn.BatchNorm()
            # batchnorm16_1_25_, output shape: {[4,56,56]}

            self.relu16_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_25_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_25_, output shape: {[256,56,56]}

            self.batchnorm17_1_25_ = gluon.nn.BatchNorm()
            # batchnorm17_1_25_, output shape: {[256,56,56]}

            self.conv15_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_26_, output shape: {[4,56,56]}

            self.batchnorm15_1_26_ = gluon.nn.BatchNorm()
            # batchnorm15_1_26_, output shape: {[4,56,56]}

            self.relu15_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_26_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_26_, output shape: {[4,56,56]}

            self.batchnorm16_1_26_ = gluon.nn.BatchNorm()
            # batchnorm16_1_26_, output shape: {[4,56,56]}

            self.relu16_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_26_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_26_, output shape: {[256,56,56]}

            self.batchnorm17_1_26_ = gluon.nn.BatchNorm()
            # batchnorm17_1_26_, output shape: {[256,56,56]}

            self.conv15_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_27_, output shape: {[4,56,56]}

            self.batchnorm15_1_27_ = gluon.nn.BatchNorm()
            # batchnorm15_1_27_, output shape: {[4,56,56]}

            self.relu15_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_27_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_27_, output shape: {[4,56,56]}

            self.batchnorm16_1_27_ = gluon.nn.BatchNorm()
            # batchnorm16_1_27_, output shape: {[4,56,56]}

            self.relu16_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_27_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_27_, output shape: {[256,56,56]}

            self.batchnorm17_1_27_ = gluon.nn.BatchNorm()
            # batchnorm17_1_27_, output shape: {[256,56,56]}

            self.conv15_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_28_, output shape: {[4,56,56]}

            self.batchnorm15_1_28_ = gluon.nn.BatchNorm()
            # batchnorm15_1_28_, output shape: {[4,56,56]}

            self.relu15_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_28_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_28_, output shape: {[4,56,56]}

            self.batchnorm16_1_28_ = gluon.nn.BatchNorm()
            # batchnorm16_1_28_, output shape: {[4,56,56]}

            self.relu16_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_28_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_28_, output shape: {[256,56,56]}

            self.batchnorm17_1_28_ = gluon.nn.BatchNorm()
            # batchnorm17_1_28_, output shape: {[256,56,56]}

            self.conv15_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_29_, output shape: {[4,56,56]}

            self.batchnorm15_1_29_ = gluon.nn.BatchNorm()
            # batchnorm15_1_29_, output shape: {[4,56,56]}

            self.relu15_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_29_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_29_, output shape: {[4,56,56]}

            self.batchnorm16_1_29_ = gluon.nn.BatchNorm()
            # batchnorm16_1_29_, output shape: {[4,56,56]}

            self.relu16_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_29_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_29_, output shape: {[256,56,56]}

            self.batchnorm17_1_29_ = gluon.nn.BatchNorm()
            # batchnorm17_1_29_, output shape: {[256,56,56]}

            self.conv15_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_30_, output shape: {[4,56,56]}

            self.batchnorm15_1_30_ = gluon.nn.BatchNorm()
            # batchnorm15_1_30_, output shape: {[4,56,56]}

            self.relu15_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_30_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_30_, output shape: {[4,56,56]}

            self.batchnorm16_1_30_ = gluon.nn.BatchNorm()
            # batchnorm16_1_30_, output shape: {[4,56,56]}

            self.relu16_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_30_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_30_, output shape: {[256,56,56]}

            self.batchnorm17_1_30_ = gluon.nn.BatchNorm()
            # batchnorm17_1_30_, output shape: {[256,56,56]}

            self.conv15_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_31_, output shape: {[4,56,56]}

            self.batchnorm15_1_31_ = gluon.nn.BatchNorm()
            # batchnorm15_1_31_, output shape: {[4,56,56]}

            self.relu15_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_31_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_31_, output shape: {[4,56,56]}

            self.batchnorm16_1_31_ = gluon.nn.BatchNorm()
            # batchnorm16_1_31_, output shape: {[4,56,56]}

            self.relu16_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_31_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_31_, output shape: {[256,56,56]}

            self.batchnorm17_1_31_ = gluon.nn.BatchNorm()
            # batchnorm17_1_31_, output shape: {[256,56,56]}

            self.conv15_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv15_1_32_, output shape: {[4,56,56]}

            self.batchnorm15_1_32_ = gluon.nn.BatchNorm()
            # batchnorm15_1_32_, output shape: {[4,56,56]}

            self.relu15_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv16_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv16_1_32_ = gluon.nn.Conv2D(channels=4,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv16_1_32_, output shape: {[4,56,56]}

            self.batchnorm16_1_32_ = gluon.nn.BatchNorm()
            # batchnorm16_1_32_, output shape: {[4,56,56]}

            self.relu16_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_32_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv17_1_32_, output shape: {[256,56,56]}

            self.batchnorm17_1_32_ = gluon.nn.BatchNorm()
            # batchnorm17_1_32_, output shape: {[256,56,56]}

            self.relu19_ = gluon.nn.Activation(activation='relu')
            self.conv21_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_1_, output shape: {[8,56,56]}

            self.batchnorm21_1_1_ = gluon.nn.BatchNorm()
            # batchnorm21_1_1_, output shape: {[8,56,56]}

            self.relu21_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_1_, output shape: {[8,28,28]}

            self.batchnorm22_1_1_ = gluon.nn.BatchNorm()
            # batchnorm22_1_1_, output shape: {[8,28,28]}

            self.relu22_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_1_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_1_, output shape: {[512,28,28]}

            self.batchnorm23_1_1_ = gluon.nn.BatchNorm()
            # batchnorm23_1_1_, output shape: {[512,28,28]}

            self.conv21_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_2_, output shape: {[8,56,56]}

            self.batchnorm21_1_2_ = gluon.nn.BatchNorm()
            # batchnorm21_1_2_, output shape: {[8,56,56]}

            self.relu21_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_2_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_2_, output shape: {[8,28,28]}

            self.batchnorm22_1_2_ = gluon.nn.BatchNorm()
            # batchnorm22_1_2_, output shape: {[8,28,28]}

            self.relu22_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_2_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_2_, output shape: {[512,28,28]}

            self.batchnorm23_1_2_ = gluon.nn.BatchNorm()
            # batchnorm23_1_2_, output shape: {[512,28,28]}

            self.conv21_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_3_, output shape: {[8,56,56]}

            self.batchnorm21_1_3_ = gluon.nn.BatchNorm()
            # batchnorm21_1_3_, output shape: {[8,56,56]}

            self.relu21_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_3_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_3_, output shape: {[8,28,28]}

            self.batchnorm22_1_3_ = gluon.nn.BatchNorm()
            # batchnorm22_1_3_, output shape: {[8,28,28]}

            self.relu22_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_3_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_3_, output shape: {[512,28,28]}

            self.batchnorm23_1_3_ = gluon.nn.BatchNorm()
            # batchnorm23_1_3_, output shape: {[512,28,28]}

            self.conv21_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_4_, output shape: {[8,56,56]}

            self.batchnorm21_1_4_ = gluon.nn.BatchNorm()
            # batchnorm21_1_4_, output shape: {[8,56,56]}

            self.relu21_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_4_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_4_, output shape: {[8,28,28]}

            self.batchnorm22_1_4_ = gluon.nn.BatchNorm()
            # batchnorm22_1_4_, output shape: {[8,28,28]}

            self.relu22_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_4_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_4_, output shape: {[512,28,28]}

            self.batchnorm23_1_4_ = gluon.nn.BatchNorm()
            # batchnorm23_1_4_, output shape: {[512,28,28]}

            self.conv21_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_5_, output shape: {[8,56,56]}

            self.batchnorm21_1_5_ = gluon.nn.BatchNorm()
            # batchnorm21_1_5_, output shape: {[8,56,56]}

            self.relu21_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_5_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_5_, output shape: {[8,28,28]}

            self.batchnorm22_1_5_ = gluon.nn.BatchNorm()
            # batchnorm22_1_5_, output shape: {[8,28,28]}

            self.relu22_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_5_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_5_, output shape: {[512,28,28]}

            self.batchnorm23_1_5_ = gluon.nn.BatchNorm()
            # batchnorm23_1_5_, output shape: {[512,28,28]}

            self.conv21_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_6_, output shape: {[8,56,56]}

            self.batchnorm21_1_6_ = gluon.nn.BatchNorm()
            # batchnorm21_1_6_, output shape: {[8,56,56]}

            self.relu21_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_6_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_6_, output shape: {[8,28,28]}

            self.batchnorm22_1_6_ = gluon.nn.BatchNorm()
            # batchnorm22_1_6_, output shape: {[8,28,28]}

            self.relu22_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_6_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_6_, output shape: {[512,28,28]}

            self.batchnorm23_1_6_ = gluon.nn.BatchNorm()
            # batchnorm23_1_6_, output shape: {[512,28,28]}

            self.conv21_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_7_, output shape: {[8,56,56]}

            self.batchnorm21_1_7_ = gluon.nn.BatchNorm()
            # batchnorm21_1_7_, output shape: {[8,56,56]}

            self.relu21_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_7_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_7_, output shape: {[8,28,28]}

            self.batchnorm22_1_7_ = gluon.nn.BatchNorm()
            # batchnorm22_1_7_, output shape: {[8,28,28]}

            self.relu22_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_7_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_7_, output shape: {[512,28,28]}

            self.batchnorm23_1_7_ = gluon.nn.BatchNorm()
            # batchnorm23_1_7_, output shape: {[512,28,28]}

            self.conv21_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_8_, output shape: {[8,56,56]}

            self.batchnorm21_1_8_ = gluon.nn.BatchNorm()
            # batchnorm21_1_8_, output shape: {[8,56,56]}

            self.relu21_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_8_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_8_, output shape: {[8,28,28]}

            self.batchnorm22_1_8_ = gluon.nn.BatchNorm()
            # batchnorm22_1_8_, output shape: {[8,28,28]}

            self.relu22_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_8_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_8_, output shape: {[512,28,28]}

            self.batchnorm23_1_8_ = gluon.nn.BatchNorm()
            # batchnorm23_1_8_, output shape: {[512,28,28]}

            self.conv21_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_9_, output shape: {[8,56,56]}

            self.batchnorm21_1_9_ = gluon.nn.BatchNorm()
            # batchnorm21_1_9_, output shape: {[8,56,56]}

            self.relu21_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_9_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_9_, output shape: {[8,28,28]}

            self.batchnorm22_1_9_ = gluon.nn.BatchNorm()
            # batchnorm22_1_9_, output shape: {[8,28,28]}

            self.relu22_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_9_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_9_, output shape: {[512,28,28]}

            self.batchnorm23_1_9_ = gluon.nn.BatchNorm()
            # batchnorm23_1_9_, output shape: {[512,28,28]}

            self.conv21_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_10_, output shape: {[8,56,56]}

            self.batchnorm21_1_10_ = gluon.nn.BatchNorm()
            # batchnorm21_1_10_, output shape: {[8,56,56]}

            self.relu21_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_10_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_10_, output shape: {[8,28,28]}

            self.batchnorm22_1_10_ = gluon.nn.BatchNorm()
            # batchnorm22_1_10_, output shape: {[8,28,28]}

            self.relu22_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_10_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_10_, output shape: {[512,28,28]}

            self.batchnorm23_1_10_ = gluon.nn.BatchNorm()
            # batchnorm23_1_10_, output shape: {[512,28,28]}

            self.conv21_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_11_, output shape: {[8,56,56]}

            self.batchnorm21_1_11_ = gluon.nn.BatchNorm()
            # batchnorm21_1_11_, output shape: {[8,56,56]}

            self.relu21_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_11_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_11_, output shape: {[8,28,28]}

            self.batchnorm22_1_11_ = gluon.nn.BatchNorm()
            # batchnorm22_1_11_, output shape: {[8,28,28]}

            self.relu22_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_11_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_11_, output shape: {[512,28,28]}

            self.batchnorm23_1_11_ = gluon.nn.BatchNorm()
            # batchnorm23_1_11_, output shape: {[512,28,28]}

            self.conv21_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_12_, output shape: {[8,56,56]}

            self.batchnorm21_1_12_ = gluon.nn.BatchNorm()
            # batchnorm21_1_12_, output shape: {[8,56,56]}

            self.relu21_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_12_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_12_, output shape: {[8,28,28]}

            self.batchnorm22_1_12_ = gluon.nn.BatchNorm()
            # batchnorm22_1_12_, output shape: {[8,28,28]}

            self.relu22_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_12_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_12_, output shape: {[512,28,28]}

            self.batchnorm23_1_12_ = gluon.nn.BatchNorm()
            # batchnorm23_1_12_, output shape: {[512,28,28]}

            self.conv21_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_13_, output shape: {[8,56,56]}

            self.batchnorm21_1_13_ = gluon.nn.BatchNorm()
            # batchnorm21_1_13_, output shape: {[8,56,56]}

            self.relu21_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_13_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_13_, output shape: {[8,28,28]}

            self.batchnorm22_1_13_ = gluon.nn.BatchNorm()
            # batchnorm22_1_13_, output shape: {[8,28,28]}

            self.relu22_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_13_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_13_, output shape: {[512,28,28]}

            self.batchnorm23_1_13_ = gluon.nn.BatchNorm()
            # batchnorm23_1_13_, output shape: {[512,28,28]}

            self.conv21_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_14_, output shape: {[8,56,56]}

            self.batchnorm21_1_14_ = gluon.nn.BatchNorm()
            # batchnorm21_1_14_, output shape: {[8,56,56]}

            self.relu21_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_14_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_14_, output shape: {[8,28,28]}

            self.batchnorm22_1_14_ = gluon.nn.BatchNorm()
            # batchnorm22_1_14_, output shape: {[8,28,28]}

            self.relu22_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_14_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_14_, output shape: {[512,28,28]}

            self.batchnorm23_1_14_ = gluon.nn.BatchNorm()
            # batchnorm23_1_14_, output shape: {[512,28,28]}

            self.conv21_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_15_, output shape: {[8,56,56]}

            self.batchnorm21_1_15_ = gluon.nn.BatchNorm()
            # batchnorm21_1_15_, output shape: {[8,56,56]}

            self.relu21_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_15_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_15_, output shape: {[8,28,28]}

            self.batchnorm22_1_15_ = gluon.nn.BatchNorm()
            # batchnorm22_1_15_, output shape: {[8,28,28]}

            self.relu22_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_15_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_15_, output shape: {[512,28,28]}

            self.batchnorm23_1_15_ = gluon.nn.BatchNorm()
            # batchnorm23_1_15_, output shape: {[512,28,28]}

            self.conv21_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_16_, output shape: {[8,56,56]}

            self.batchnorm21_1_16_ = gluon.nn.BatchNorm()
            # batchnorm21_1_16_, output shape: {[8,56,56]}

            self.relu21_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_16_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_16_, output shape: {[8,28,28]}

            self.batchnorm22_1_16_ = gluon.nn.BatchNorm()
            # batchnorm22_1_16_, output shape: {[8,28,28]}

            self.relu22_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_16_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_16_, output shape: {[512,28,28]}

            self.batchnorm23_1_16_ = gluon.nn.BatchNorm()
            # batchnorm23_1_16_, output shape: {[512,28,28]}

            self.conv21_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_17_, output shape: {[8,56,56]}

            self.batchnorm21_1_17_ = gluon.nn.BatchNorm()
            # batchnorm21_1_17_, output shape: {[8,56,56]}

            self.relu21_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_17_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_17_, output shape: {[8,28,28]}

            self.batchnorm22_1_17_ = gluon.nn.BatchNorm()
            # batchnorm22_1_17_, output shape: {[8,28,28]}

            self.relu22_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_17_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_17_, output shape: {[512,28,28]}

            self.batchnorm23_1_17_ = gluon.nn.BatchNorm()
            # batchnorm23_1_17_, output shape: {[512,28,28]}

            self.conv21_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_18_, output shape: {[8,56,56]}

            self.batchnorm21_1_18_ = gluon.nn.BatchNorm()
            # batchnorm21_1_18_, output shape: {[8,56,56]}

            self.relu21_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_18_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_18_, output shape: {[8,28,28]}

            self.batchnorm22_1_18_ = gluon.nn.BatchNorm()
            # batchnorm22_1_18_, output shape: {[8,28,28]}

            self.relu22_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_18_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_18_, output shape: {[512,28,28]}

            self.batchnorm23_1_18_ = gluon.nn.BatchNorm()
            # batchnorm23_1_18_, output shape: {[512,28,28]}

            self.conv21_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_19_, output shape: {[8,56,56]}

            self.batchnorm21_1_19_ = gluon.nn.BatchNorm()
            # batchnorm21_1_19_, output shape: {[8,56,56]}

            self.relu21_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_19_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_19_, output shape: {[8,28,28]}

            self.batchnorm22_1_19_ = gluon.nn.BatchNorm()
            # batchnorm22_1_19_, output shape: {[8,28,28]}

            self.relu22_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_19_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_19_, output shape: {[512,28,28]}

            self.batchnorm23_1_19_ = gluon.nn.BatchNorm()
            # batchnorm23_1_19_, output shape: {[512,28,28]}

            self.conv21_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_20_, output shape: {[8,56,56]}

            self.batchnorm21_1_20_ = gluon.nn.BatchNorm()
            # batchnorm21_1_20_, output shape: {[8,56,56]}

            self.relu21_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_20_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_20_, output shape: {[8,28,28]}

            self.batchnorm22_1_20_ = gluon.nn.BatchNorm()
            # batchnorm22_1_20_, output shape: {[8,28,28]}

            self.relu22_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_20_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_20_, output shape: {[512,28,28]}

            self.batchnorm23_1_20_ = gluon.nn.BatchNorm()
            # batchnorm23_1_20_, output shape: {[512,28,28]}

            self.conv21_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_21_, output shape: {[8,56,56]}

            self.batchnorm21_1_21_ = gluon.nn.BatchNorm()
            # batchnorm21_1_21_, output shape: {[8,56,56]}

            self.relu21_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_21_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_21_, output shape: {[8,28,28]}

            self.batchnorm22_1_21_ = gluon.nn.BatchNorm()
            # batchnorm22_1_21_, output shape: {[8,28,28]}

            self.relu22_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_21_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_21_, output shape: {[512,28,28]}

            self.batchnorm23_1_21_ = gluon.nn.BatchNorm()
            # batchnorm23_1_21_, output shape: {[512,28,28]}

            self.conv21_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_22_, output shape: {[8,56,56]}

            self.batchnorm21_1_22_ = gluon.nn.BatchNorm()
            # batchnorm21_1_22_, output shape: {[8,56,56]}

            self.relu21_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_22_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_22_, output shape: {[8,28,28]}

            self.batchnorm22_1_22_ = gluon.nn.BatchNorm()
            # batchnorm22_1_22_, output shape: {[8,28,28]}

            self.relu22_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_22_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_22_, output shape: {[512,28,28]}

            self.batchnorm23_1_22_ = gluon.nn.BatchNorm()
            # batchnorm23_1_22_, output shape: {[512,28,28]}

            self.conv21_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_23_, output shape: {[8,56,56]}

            self.batchnorm21_1_23_ = gluon.nn.BatchNorm()
            # batchnorm21_1_23_, output shape: {[8,56,56]}

            self.relu21_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_23_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_23_, output shape: {[8,28,28]}

            self.batchnorm22_1_23_ = gluon.nn.BatchNorm()
            # batchnorm22_1_23_, output shape: {[8,28,28]}

            self.relu22_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_23_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_23_, output shape: {[512,28,28]}

            self.batchnorm23_1_23_ = gluon.nn.BatchNorm()
            # batchnorm23_1_23_, output shape: {[512,28,28]}

            self.conv21_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_24_, output shape: {[8,56,56]}

            self.batchnorm21_1_24_ = gluon.nn.BatchNorm()
            # batchnorm21_1_24_, output shape: {[8,56,56]}

            self.relu21_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_24_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_24_, output shape: {[8,28,28]}

            self.batchnorm22_1_24_ = gluon.nn.BatchNorm()
            # batchnorm22_1_24_, output shape: {[8,28,28]}

            self.relu22_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_24_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_24_, output shape: {[512,28,28]}

            self.batchnorm23_1_24_ = gluon.nn.BatchNorm()
            # batchnorm23_1_24_, output shape: {[512,28,28]}

            self.conv21_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_25_, output shape: {[8,56,56]}

            self.batchnorm21_1_25_ = gluon.nn.BatchNorm()
            # batchnorm21_1_25_, output shape: {[8,56,56]}

            self.relu21_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_25_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_25_, output shape: {[8,28,28]}

            self.batchnorm22_1_25_ = gluon.nn.BatchNorm()
            # batchnorm22_1_25_, output shape: {[8,28,28]}

            self.relu22_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_25_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_25_, output shape: {[512,28,28]}

            self.batchnorm23_1_25_ = gluon.nn.BatchNorm()
            # batchnorm23_1_25_, output shape: {[512,28,28]}

            self.conv21_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_26_, output shape: {[8,56,56]}

            self.batchnorm21_1_26_ = gluon.nn.BatchNorm()
            # batchnorm21_1_26_, output shape: {[8,56,56]}

            self.relu21_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_26_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_26_, output shape: {[8,28,28]}

            self.batchnorm22_1_26_ = gluon.nn.BatchNorm()
            # batchnorm22_1_26_, output shape: {[8,28,28]}

            self.relu22_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_26_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_26_, output shape: {[512,28,28]}

            self.batchnorm23_1_26_ = gluon.nn.BatchNorm()
            # batchnorm23_1_26_, output shape: {[512,28,28]}

            self.conv21_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_27_, output shape: {[8,56,56]}

            self.batchnorm21_1_27_ = gluon.nn.BatchNorm()
            # batchnorm21_1_27_, output shape: {[8,56,56]}

            self.relu21_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_27_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_27_, output shape: {[8,28,28]}

            self.batchnorm22_1_27_ = gluon.nn.BatchNorm()
            # batchnorm22_1_27_, output shape: {[8,28,28]}

            self.relu22_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_27_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_27_, output shape: {[512,28,28]}

            self.batchnorm23_1_27_ = gluon.nn.BatchNorm()
            # batchnorm23_1_27_, output shape: {[512,28,28]}

            self.conv21_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_28_, output shape: {[8,56,56]}

            self.batchnorm21_1_28_ = gluon.nn.BatchNorm()
            # batchnorm21_1_28_, output shape: {[8,56,56]}

            self.relu21_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_28_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_28_, output shape: {[8,28,28]}

            self.batchnorm22_1_28_ = gluon.nn.BatchNorm()
            # batchnorm22_1_28_, output shape: {[8,28,28]}

            self.relu22_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_28_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_28_, output shape: {[512,28,28]}

            self.batchnorm23_1_28_ = gluon.nn.BatchNorm()
            # batchnorm23_1_28_, output shape: {[512,28,28]}

            self.conv21_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_29_, output shape: {[8,56,56]}

            self.batchnorm21_1_29_ = gluon.nn.BatchNorm()
            # batchnorm21_1_29_, output shape: {[8,56,56]}

            self.relu21_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_29_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_29_, output shape: {[8,28,28]}

            self.batchnorm22_1_29_ = gluon.nn.BatchNorm()
            # batchnorm22_1_29_, output shape: {[8,28,28]}

            self.relu22_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_29_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_29_, output shape: {[512,28,28]}

            self.batchnorm23_1_29_ = gluon.nn.BatchNorm()
            # batchnorm23_1_29_, output shape: {[512,28,28]}

            self.conv21_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_30_, output shape: {[8,56,56]}

            self.batchnorm21_1_30_ = gluon.nn.BatchNorm()
            # batchnorm21_1_30_, output shape: {[8,56,56]}

            self.relu21_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_30_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_30_, output shape: {[8,28,28]}

            self.batchnorm22_1_30_ = gluon.nn.BatchNorm()
            # batchnorm22_1_30_, output shape: {[8,28,28]}

            self.relu22_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_30_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_30_, output shape: {[512,28,28]}

            self.batchnorm23_1_30_ = gluon.nn.BatchNorm()
            # batchnorm23_1_30_, output shape: {[512,28,28]}

            self.conv21_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_31_, output shape: {[8,56,56]}

            self.batchnorm21_1_31_ = gluon.nn.BatchNorm()
            # batchnorm21_1_31_, output shape: {[8,56,56]}

            self.relu21_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_31_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_31_, output shape: {[8,28,28]}

            self.batchnorm22_1_31_ = gluon.nn.BatchNorm()
            # batchnorm22_1_31_, output shape: {[8,28,28]}

            self.relu22_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_31_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_31_, output shape: {[512,28,28]}

            self.batchnorm23_1_31_ = gluon.nn.BatchNorm()
            # batchnorm23_1_31_, output shape: {[512,28,28]}

            self.conv21_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv21_1_32_, output shape: {[8,56,56]}

            self.batchnorm21_1_32_ = gluon.nn.BatchNorm()
            # batchnorm21_1_32_, output shape: {[8,56,56]}

            self.relu21_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv22_1_32_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv22_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv22_1_32_, output shape: {[8,28,28]}

            self.batchnorm22_1_32_ = gluon.nn.BatchNorm()
            # batchnorm22_1_32_, output shape: {[8,28,28]}

            self.relu22_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_32_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv23_1_32_, output shape: {[512,28,28]}

            self.batchnorm23_1_32_ = gluon.nn.BatchNorm()
            # batchnorm23_1_32_, output shape: {[512,28,28]}

            self.conv20_2_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv20_2_, output shape: {[512,28,28]}

            self.batchnorm20_2_ = gluon.nn.BatchNorm()
            # batchnorm20_2_, output shape: {[512,28,28]}

            self.relu25_ = gluon.nn.Activation(activation='relu')
            self.conv27_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_1_, output shape: {[8,28,28]}

            self.batchnorm27_1_1_ = gluon.nn.BatchNorm()
            # batchnorm27_1_1_, output shape: {[8,28,28]}

            self.relu27_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_1_, output shape: {[8,28,28]}

            self.batchnorm28_1_1_ = gluon.nn.BatchNorm()
            # batchnorm28_1_1_, output shape: {[8,28,28]}

            self.relu28_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_1_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_1_, output shape: {[512,28,28]}

            self.batchnorm29_1_1_ = gluon.nn.BatchNorm()
            # batchnorm29_1_1_, output shape: {[512,28,28]}

            self.conv27_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_2_, output shape: {[8,28,28]}

            self.batchnorm27_1_2_ = gluon.nn.BatchNorm()
            # batchnorm27_1_2_, output shape: {[8,28,28]}

            self.relu27_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_2_, output shape: {[8,28,28]}

            self.batchnorm28_1_2_ = gluon.nn.BatchNorm()
            # batchnorm28_1_2_, output shape: {[8,28,28]}

            self.relu28_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_2_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_2_, output shape: {[512,28,28]}

            self.batchnorm29_1_2_ = gluon.nn.BatchNorm()
            # batchnorm29_1_2_, output shape: {[512,28,28]}

            self.conv27_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_3_, output shape: {[8,28,28]}

            self.batchnorm27_1_3_ = gluon.nn.BatchNorm()
            # batchnorm27_1_3_, output shape: {[8,28,28]}

            self.relu27_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_3_, output shape: {[8,28,28]}

            self.batchnorm28_1_3_ = gluon.nn.BatchNorm()
            # batchnorm28_1_3_, output shape: {[8,28,28]}

            self.relu28_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_3_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_3_, output shape: {[512,28,28]}

            self.batchnorm29_1_3_ = gluon.nn.BatchNorm()
            # batchnorm29_1_3_, output shape: {[512,28,28]}

            self.conv27_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_4_, output shape: {[8,28,28]}

            self.batchnorm27_1_4_ = gluon.nn.BatchNorm()
            # batchnorm27_1_4_, output shape: {[8,28,28]}

            self.relu27_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_4_, output shape: {[8,28,28]}

            self.batchnorm28_1_4_ = gluon.nn.BatchNorm()
            # batchnorm28_1_4_, output shape: {[8,28,28]}

            self.relu28_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_4_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_4_, output shape: {[512,28,28]}

            self.batchnorm29_1_4_ = gluon.nn.BatchNorm()
            # batchnorm29_1_4_, output shape: {[512,28,28]}

            self.conv27_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_5_, output shape: {[8,28,28]}

            self.batchnorm27_1_5_ = gluon.nn.BatchNorm()
            # batchnorm27_1_5_, output shape: {[8,28,28]}

            self.relu27_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_5_, output shape: {[8,28,28]}

            self.batchnorm28_1_5_ = gluon.nn.BatchNorm()
            # batchnorm28_1_5_, output shape: {[8,28,28]}

            self.relu28_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_5_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_5_, output shape: {[512,28,28]}

            self.batchnorm29_1_5_ = gluon.nn.BatchNorm()
            # batchnorm29_1_5_, output shape: {[512,28,28]}

            self.conv27_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_6_, output shape: {[8,28,28]}

            self.batchnorm27_1_6_ = gluon.nn.BatchNorm()
            # batchnorm27_1_6_, output shape: {[8,28,28]}

            self.relu27_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_6_, output shape: {[8,28,28]}

            self.batchnorm28_1_6_ = gluon.nn.BatchNorm()
            # batchnorm28_1_6_, output shape: {[8,28,28]}

            self.relu28_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_6_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_6_, output shape: {[512,28,28]}

            self.batchnorm29_1_6_ = gluon.nn.BatchNorm()
            # batchnorm29_1_6_, output shape: {[512,28,28]}

            self.conv27_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_7_, output shape: {[8,28,28]}

            self.batchnorm27_1_7_ = gluon.nn.BatchNorm()
            # batchnorm27_1_7_, output shape: {[8,28,28]}

            self.relu27_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_7_, output shape: {[8,28,28]}

            self.batchnorm28_1_7_ = gluon.nn.BatchNorm()
            # batchnorm28_1_7_, output shape: {[8,28,28]}

            self.relu28_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_7_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_7_, output shape: {[512,28,28]}

            self.batchnorm29_1_7_ = gluon.nn.BatchNorm()
            # batchnorm29_1_7_, output shape: {[512,28,28]}

            self.conv27_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_8_, output shape: {[8,28,28]}

            self.batchnorm27_1_8_ = gluon.nn.BatchNorm()
            # batchnorm27_1_8_, output shape: {[8,28,28]}

            self.relu27_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_8_, output shape: {[8,28,28]}

            self.batchnorm28_1_8_ = gluon.nn.BatchNorm()
            # batchnorm28_1_8_, output shape: {[8,28,28]}

            self.relu28_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_8_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_8_, output shape: {[512,28,28]}

            self.batchnorm29_1_8_ = gluon.nn.BatchNorm()
            # batchnorm29_1_8_, output shape: {[512,28,28]}

            self.conv27_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_9_, output shape: {[8,28,28]}

            self.batchnorm27_1_9_ = gluon.nn.BatchNorm()
            # batchnorm27_1_9_, output shape: {[8,28,28]}

            self.relu27_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_9_, output shape: {[8,28,28]}

            self.batchnorm28_1_9_ = gluon.nn.BatchNorm()
            # batchnorm28_1_9_, output shape: {[8,28,28]}

            self.relu28_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_9_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_9_, output shape: {[512,28,28]}

            self.batchnorm29_1_9_ = gluon.nn.BatchNorm()
            # batchnorm29_1_9_, output shape: {[512,28,28]}

            self.conv27_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_10_, output shape: {[8,28,28]}

            self.batchnorm27_1_10_ = gluon.nn.BatchNorm()
            # batchnorm27_1_10_, output shape: {[8,28,28]}

            self.relu27_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_10_, output shape: {[8,28,28]}

            self.batchnorm28_1_10_ = gluon.nn.BatchNorm()
            # batchnorm28_1_10_, output shape: {[8,28,28]}

            self.relu28_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_10_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_10_, output shape: {[512,28,28]}

            self.batchnorm29_1_10_ = gluon.nn.BatchNorm()
            # batchnorm29_1_10_, output shape: {[512,28,28]}

            self.conv27_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_11_, output shape: {[8,28,28]}

            self.batchnorm27_1_11_ = gluon.nn.BatchNorm()
            # batchnorm27_1_11_, output shape: {[8,28,28]}

            self.relu27_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_11_, output shape: {[8,28,28]}

            self.batchnorm28_1_11_ = gluon.nn.BatchNorm()
            # batchnorm28_1_11_, output shape: {[8,28,28]}

            self.relu28_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_11_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_11_, output shape: {[512,28,28]}

            self.batchnorm29_1_11_ = gluon.nn.BatchNorm()
            # batchnorm29_1_11_, output shape: {[512,28,28]}

            self.conv27_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_12_, output shape: {[8,28,28]}

            self.batchnorm27_1_12_ = gluon.nn.BatchNorm()
            # batchnorm27_1_12_, output shape: {[8,28,28]}

            self.relu27_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_12_, output shape: {[8,28,28]}

            self.batchnorm28_1_12_ = gluon.nn.BatchNorm()
            # batchnorm28_1_12_, output shape: {[8,28,28]}

            self.relu28_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_12_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_12_, output shape: {[512,28,28]}

            self.batchnorm29_1_12_ = gluon.nn.BatchNorm()
            # batchnorm29_1_12_, output shape: {[512,28,28]}

            self.conv27_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_13_, output shape: {[8,28,28]}

            self.batchnorm27_1_13_ = gluon.nn.BatchNorm()
            # batchnorm27_1_13_, output shape: {[8,28,28]}

            self.relu27_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_13_, output shape: {[8,28,28]}

            self.batchnorm28_1_13_ = gluon.nn.BatchNorm()
            # batchnorm28_1_13_, output shape: {[8,28,28]}

            self.relu28_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_13_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_13_, output shape: {[512,28,28]}

            self.batchnorm29_1_13_ = gluon.nn.BatchNorm()
            # batchnorm29_1_13_, output shape: {[512,28,28]}

            self.conv27_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_14_, output shape: {[8,28,28]}

            self.batchnorm27_1_14_ = gluon.nn.BatchNorm()
            # batchnorm27_1_14_, output shape: {[8,28,28]}

            self.relu27_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_14_, output shape: {[8,28,28]}

            self.batchnorm28_1_14_ = gluon.nn.BatchNorm()
            # batchnorm28_1_14_, output shape: {[8,28,28]}

            self.relu28_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_14_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_14_, output shape: {[512,28,28]}

            self.batchnorm29_1_14_ = gluon.nn.BatchNorm()
            # batchnorm29_1_14_, output shape: {[512,28,28]}

            self.conv27_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_15_, output shape: {[8,28,28]}

            self.batchnorm27_1_15_ = gluon.nn.BatchNorm()
            # batchnorm27_1_15_, output shape: {[8,28,28]}

            self.relu27_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_15_, output shape: {[8,28,28]}

            self.batchnorm28_1_15_ = gluon.nn.BatchNorm()
            # batchnorm28_1_15_, output shape: {[8,28,28]}

            self.relu28_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_15_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_15_, output shape: {[512,28,28]}

            self.batchnorm29_1_15_ = gluon.nn.BatchNorm()
            # batchnorm29_1_15_, output shape: {[512,28,28]}

            self.conv27_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_16_, output shape: {[8,28,28]}

            self.batchnorm27_1_16_ = gluon.nn.BatchNorm()
            # batchnorm27_1_16_, output shape: {[8,28,28]}

            self.relu27_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_16_, output shape: {[8,28,28]}

            self.batchnorm28_1_16_ = gluon.nn.BatchNorm()
            # batchnorm28_1_16_, output shape: {[8,28,28]}

            self.relu28_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_16_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_16_, output shape: {[512,28,28]}

            self.batchnorm29_1_16_ = gluon.nn.BatchNorm()
            # batchnorm29_1_16_, output shape: {[512,28,28]}

            self.conv27_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_17_, output shape: {[8,28,28]}

            self.batchnorm27_1_17_ = gluon.nn.BatchNorm()
            # batchnorm27_1_17_, output shape: {[8,28,28]}

            self.relu27_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_17_, output shape: {[8,28,28]}

            self.batchnorm28_1_17_ = gluon.nn.BatchNorm()
            # batchnorm28_1_17_, output shape: {[8,28,28]}

            self.relu28_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_17_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_17_, output shape: {[512,28,28]}

            self.batchnorm29_1_17_ = gluon.nn.BatchNorm()
            # batchnorm29_1_17_, output shape: {[512,28,28]}

            self.conv27_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_18_, output shape: {[8,28,28]}

            self.batchnorm27_1_18_ = gluon.nn.BatchNorm()
            # batchnorm27_1_18_, output shape: {[8,28,28]}

            self.relu27_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_18_, output shape: {[8,28,28]}

            self.batchnorm28_1_18_ = gluon.nn.BatchNorm()
            # batchnorm28_1_18_, output shape: {[8,28,28]}

            self.relu28_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_18_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_18_, output shape: {[512,28,28]}

            self.batchnorm29_1_18_ = gluon.nn.BatchNorm()
            # batchnorm29_1_18_, output shape: {[512,28,28]}

            self.conv27_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_19_, output shape: {[8,28,28]}

            self.batchnorm27_1_19_ = gluon.nn.BatchNorm()
            # batchnorm27_1_19_, output shape: {[8,28,28]}

            self.relu27_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_19_, output shape: {[8,28,28]}

            self.batchnorm28_1_19_ = gluon.nn.BatchNorm()
            # batchnorm28_1_19_, output shape: {[8,28,28]}

            self.relu28_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_19_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_19_, output shape: {[512,28,28]}

            self.batchnorm29_1_19_ = gluon.nn.BatchNorm()
            # batchnorm29_1_19_, output shape: {[512,28,28]}

            self.conv27_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_20_, output shape: {[8,28,28]}

            self.batchnorm27_1_20_ = gluon.nn.BatchNorm()
            # batchnorm27_1_20_, output shape: {[8,28,28]}

            self.relu27_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_20_, output shape: {[8,28,28]}

            self.batchnorm28_1_20_ = gluon.nn.BatchNorm()
            # batchnorm28_1_20_, output shape: {[8,28,28]}

            self.relu28_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_20_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_20_, output shape: {[512,28,28]}

            self.batchnorm29_1_20_ = gluon.nn.BatchNorm()
            # batchnorm29_1_20_, output shape: {[512,28,28]}

            self.conv27_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_21_, output shape: {[8,28,28]}

            self.batchnorm27_1_21_ = gluon.nn.BatchNorm()
            # batchnorm27_1_21_, output shape: {[8,28,28]}

            self.relu27_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_21_, output shape: {[8,28,28]}

            self.batchnorm28_1_21_ = gluon.nn.BatchNorm()
            # batchnorm28_1_21_, output shape: {[8,28,28]}

            self.relu28_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_21_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_21_, output shape: {[512,28,28]}

            self.batchnorm29_1_21_ = gluon.nn.BatchNorm()
            # batchnorm29_1_21_, output shape: {[512,28,28]}

            self.conv27_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_22_, output shape: {[8,28,28]}

            self.batchnorm27_1_22_ = gluon.nn.BatchNorm()
            # batchnorm27_1_22_, output shape: {[8,28,28]}

            self.relu27_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_22_, output shape: {[8,28,28]}

            self.batchnorm28_1_22_ = gluon.nn.BatchNorm()
            # batchnorm28_1_22_, output shape: {[8,28,28]}

            self.relu28_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_22_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_22_, output shape: {[512,28,28]}

            self.batchnorm29_1_22_ = gluon.nn.BatchNorm()
            # batchnorm29_1_22_, output shape: {[512,28,28]}

            self.conv27_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_23_, output shape: {[8,28,28]}

            self.batchnorm27_1_23_ = gluon.nn.BatchNorm()
            # batchnorm27_1_23_, output shape: {[8,28,28]}

            self.relu27_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_23_, output shape: {[8,28,28]}

            self.batchnorm28_1_23_ = gluon.nn.BatchNorm()
            # batchnorm28_1_23_, output shape: {[8,28,28]}

            self.relu28_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_23_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_23_, output shape: {[512,28,28]}

            self.batchnorm29_1_23_ = gluon.nn.BatchNorm()
            # batchnorm29_1_23_, output shape: {[512,28,28]}

            self.conv27_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_24_, output shape: {[8,28,28]}

            self.batchnorm27_1_24_ = gluon.nn.BatchNorm()
            # batchnorm27_1_24_, output shape: {[8,28,28]}

            self.relu27_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_24_, output shape: {[8,28,28]}

            self.batchnorm28_1_24_ = gluon.nn.BatchNorm()
            # batchnorm28_1_24_, output shape: {[8,28,28]}

            self.relu28_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_24_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_24_, output shape: {[512,28,28]}

            self.batchnorm29_1_24_ = gluon.nn.BatchNorm()
            # batchnorm29_1_24_, output shape: {[512,28,28]}

            self.conv27_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_25_, output shape: {[8,28,28]}

            self.batchnorm27_1_25_ = gluon.nn.BatchNorm()
            # batchnorm27_1_25_, output shape: {[8,28,28]}

            self.relu27_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_25_, output shape: {[8,28,28]}

            self.batchnorm28_1_25_ = gluon.nn.BatchNorm()
            # batchnorm28_1_25_, output shape: {[8,28,28]}

            self.relu28_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_25_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_25_, output shape: {[512,28,28]}

            self.batchnorm29_1_25_ = gluon.nn.BatchNorm()
            # batchnorm29_1_25_, output shape: {[512,28,28]}

            self.conv27_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_26_, output shape: {[8,28,28]}

            self.batchnorm27_1_26_ = gluon.nn.BatchNorm()
            # batchnorm27_1_26_, output shape: {[8,28,28]}

            self.relu27_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_26_, output shape: {[8,28,28]}

            self.batchnorm28_1_26_ = gluon.nn.BatchNorm()
            # batchnorm28_1_26_, output shape: {[8,28,28]}

            self.relu28_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_26_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_26_, output shape: {[512,28,28]}

            self.batchnorm29_1_26_ = gluon.nn.BatchNorm()
            # batchnorm29_1_26_, output shape: {[512,28,28]}

            self.conv27_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_27_, output shape: {[8,28,28]}

            self.batchnorm27_1_27_ = gluon.nn.BatchNorm()
            # batchnorm27_1_27_, output shape: {[8,28,28]}

            self.relu27_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_27_, output shape: {[8,28,28]}

            self.batchnorm28_1_27_ = gluon.nn.BatchNorm()
            # batchnorm28_1_27_, output shape: {[8,28,28]}

            self.relu28_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_27_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_27_, output shape: {[512,28,28]}

            self.batchnorm29_1_27_ = gluon.nn.BatchNorm()
            # batchnorm29_1_27_, output shape: {[512,28,28]}

            self.conv27_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_28_, output shape: {[8,28,28]}

            self.batchnorm27_1_28_ = gluon.nn.BatchNorm()
            # batchnorm27_1_28_, output shape: {[8,28,28]}

            self.relu27_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_28_, output shape: {[8,28,28]}

            self.batchnorm28_1_28_ = gluon.nn.BatchNorm()
            # batchnorm28_1_28_, output shape: {[8,28,28]}

            self.relu28_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_28_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_28_, output shape: {[512,28,28]}

            self.batchnorm29_1_28_ = gluon.nn.BatchNorm()
            # batchnorm29_1_28_, output shape: {[512,28,28]}

            self.conv27_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_29_, output shape: {[8,28,28]}

            self.batchnorm27_1_29_ = gluon.nn.BatchNorm()
            # batchnorm27_1_29_, output shape: {[8,28,28]}

            self.relu27_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_29_, output shape: {[8,28,28]}

            self.batchnorm28_1_29_ = gluon.nn.BatchNorm()
            # batchnorm28_1_29_, output shape: {[8,28,28]}

            self.relu28_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_29_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_29_, output shape: {[512,28,28]}

            self.batchnorm29_1_29_ = gluon.nn.BatchNorm()
            # batchnorm29_1_29_, output shape: {[512,28,28]}

            self.conv27_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_30_, output shape: {[8,28,28]}

            self.batchnorm27_1_30_ = gluon.nn.BatchNorm()
            # batchnorm27_1_30_, output shape: {[8,28,28]}

            self.relu27_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_30_, output shape: {[8,28,28]}

            self.batchnorm28_1_30_ = gluon.nn.BatchNorm()
            # batchnorm28_1_30_, output shape: {[8,28,28]}

            self.relu28_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_30_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_30_, output shape: {[512,28,28]}

            self.batchnorm29_1_30_ = gluon.nn.BatchNorm()
            # batchnorm29_1_30_, output shape: {[512,28,28]}

            self.conv27_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_31_, output shape: {[8,28,28]}

            self.batchnorm27_1_31_ = gluon.nn.BatchNorm()
            # batchnorm27_1_31_, output shape: {[8,28,28]}

            self.relu27_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_31_, output shape: {[8,28,28]}

            self.batchnorm28_1_31_ = gluon.nn.BatchNorm()
            # batchnorm28_1_31_, output shape: {[8,28,28]}

            self.relu28_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_31_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_31_, output shape: {[512,28,28]}

            self.batchnorm29_1_31_ = gluon.nn.BatchNorm()
            # batchnorm29_1_31_, output shape: {[512,28,28]}

            self.conv27_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv27_1_32_, output shape: {[8,28,28]}

            self.batchnorm27_1_32_ = gluon.nn.BatchNorm()
            # batchnorm27_1_32_, output shape: {[8,28,28]}

            self.relu27_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv28_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv28_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv28_1_32_, output shape: {[8,28,28]}

            self.batchnorm28_1_32_ = gluon.nn.BatchNorm()
            # batchnorm28_1_32_, output shape: {[8,28,28]}

            self.relu28_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_32_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv29_1_32_, output shape: {[512,28,28]}

            self.batchnorm29_1_32_ = gluon.nn.BatchNorm()
            # batchnorm29_1_32_, output shape: {[512,28,28]}

            self.relu31_ = gluon.nn.Activation(activation='relu')
            self.conv33_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_1_, output shape: {[8,28,28]}

            self.batchnorm33_1_1_ = gluon.nn.BatchNorm()
            # batchnorm33_1_1_, output shape: {[8,28,28]}

            self.relu33_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_1_, output shape: {[8,28,28]}

            self.batchnorm34_1_1_ = gluon.nn.BatchNorm()
            # batchnorm34_1_1_, output shape: {[8,28,28]}

            self.relu34_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_1_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_1_, output shape: {[512,28,28]}

            self.batchnorm35_1_1_ = gluon.nn.BatchNorm()
            # batchnorm35_1_1_, output shape: {[512,28,28]}

            self.conv33_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_2_, output shape: {[8,28,28]}

            self.batchnorm33_1_2_ = gluon.nn.BatchNorm()
            # batchnorm33_1_2_, output shape: {[8,28,28]}

            self.relu33_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_2_, output shape: {[8,28,28]}

            self.batchnorm34_1_2_ = gluon.nn.BatchNorm()
            # batchnorm34_1_2_, output shape: {[8,28,28]}

            self.relu34_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_2_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_2_, output shape: {[512,28,28]}

            self.batchnorm35_1_2_ = gluon.nn.BatchNorm()
            # batchnorm35_1_2_, output shape: {[512,28,28]}

            self.conv33_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_3_, output shape: {[8,28,28]}

            self.batchnorm33_1_3_ = gluon.nn.BatchNorm()
            # batchnorm33_1_3_, output shape: {[8,28,28]}

            self.relu33_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_3_, output shape: {[8,28,28]}

            self.batchnorm34_1_3_ = gluon.nn.BatchNorm()
            # batchnorm34_1_3_, output shape: {[8,28,28]}

            self.relu34_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_3_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_3_, output shape: {[512,28,28]}

            self.batchnorm35_1_3_ = gluon.nn.BatchNorm()
            # batchnorm35_1_3_, output shape: {[512,28,28]}

            self.conv33_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_4_, output shape: {[8,28,28]}

            self.batchnorm33_1_4_ = gluon.nn.BatchNorm()
            # batchnorm33_1_4_, output shape: {[8,28,28]}

            self.relu33_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_4_, output shape: {[8,28,28]}

            self.batchnorm34_1_4_ = gluon.nn.BatchNorm()
            # batchnorm34_1_4_, output shape: {[8,28,28]}

            self.relu34_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_4_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_4_, output shape: {[512,28,28]}

            self.batchnorm35_1_4_ = gluon.nn.BatchNorm()
            # batchnorm35_1_4_, output shape: {[512,28,28]}

            self.conv33_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_5_, output shape: {[8,28,28]}

            self.batchnorm33_1_5_ = gluon.nn.BatchNorm()
            # batchnorm33_1_5_, output shape: {[8,28,28]}

            self.relu33_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_5_, output shape: {[8,28,28]}

            self.batchnorm34_1_5_ = gluon.nn.BatchNorm()
            # batchnorm34_1_5_, output shape: {[8,28,28]}

            self.relu34_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_5_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_5_, output shape: {[512,28,28]}

            self.batchnorm35_1_5_ = gluon.nn.BatchNorm()
            # batchnorm35_1_5_, output shape: {[512,28,28]}

            self.conv33_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_6_, output shape: {[8,28,28]}

            self.batchnorm33_1_6_ = gluon.nn.BatchNorm()
            # batchnorm33_1_6_, output shape: {[8,28,28]}

            self.relu33_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_6_, output shape: {[8,28,28]}

            self.batchnorm34_1_6_ = gluon.nn.BatchNorm()
            # batchnorm34_1_6_, output shape: {[8,28,28]}

            self.relu34_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_6_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_6_, output shape: {[512,28,28]}

            self.batchnorm35_1_6_ = gluon.nn.BatchNorm()
            # batchnorm35_1_6_, output shape: {[512,28,28]}

            self.conv33_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_7_, output shape: {[8,28,28]}

            self.batchnorm33_1_7_ = gluon.nn.BatchNorm()
            # batchnorm33_1_7_, output shape: {[8,28,28]}

            self.relu33_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_7_, output shape: {[8,28,28]}

            self.batchnorm34_1_7_ = gluon.nn.BatchNorm()
            # batchnorm34_1_7_, output shape: {[8,28,28]}

            self.relu34_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_7_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_7_, output shape: {[512,28,28]}

            self.batchnorm35_1_7_ = gluon.nn.BatchNorm()
            # batchnorm35_1_7_, output shape: {[512,28,28]}

            self.conv33_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_8_, output shape: {[8,28,28]}

            self.batchnorm33_1_8_ = gluon.nn.BatchNorm()
            # batchnorm33_1_8_, output shape: {[8,28,28]}

            self.relu33_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_8_, output shape: {[8,28,28]}

            self.batchnorm34_1_8_ = gluon.nn.BatchNorm()
            # batchnorm34_1_8_, output shape: {[8,28,28]}

            self.relu34_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_8_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_8_, output shape: {[512,28,28]}

            self.batchnorm35_1_8_ = gluon.nn.BatchNorm()
            # batchnorm35_1_8_, output shape: {[512,28,28]}

            self.conv33_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_9_, output shape: {[8,28,28]}

            self.batchnorm33_1_9_ = gluon.nn.BatchNorm()
            # batchnorm33_1_9_, output shape: {[8,28,28]}

            self.relu33_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_9_, output shape: {[8,28,28]}

            self.batchnorm34_1_9_ = gluon.nn.BatchNorm()
            # batchnorm34_1_9_, output shape: {[8,28,28]}

            self.relu34_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_9_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_9_, output shape: {[512,28,28]}

            self.batchnorm35_1_9_ = gluon.nn.BatchNorm()
            # batchnorm35_1_9_, output shape: {[512,28,28]}

            self.conv33_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_10_, output shape: {[8,28,28]}

            self.batchnorm33_1_10_ = gluon.nn.BatchNorm()
            # batchnorm33_1_10_, output shape: {[8,28,28]}

            self.relu33_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_10_, output shape: {[8,28,28]}

            self.batchnorm34_1_10_ = gluon.nn.BatchNorm()
            # batchnorm34_1_10_, output shape: {[8,28,28]}

            self.relu34_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_10_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_10_, output shape: {[512,28,28]}

            self.batchnorm35_1_10_ = gluon.nn.BatchNorm()
            # batchnorm35_1_10_, output shape: {[512,28,28]}

            self.conv33_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_11_, output shape: {[8,28,28]}

            self.batchnorm33_1_11_ = gluon.nn.BatchNorm()
            # batchnorm33_1_11_, output shape: {[8,28,28]}

            self.relu33_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_11_, output shape: {[8,28,28]}

            self.batchnorm34_1_11_ = gluon.nn.BatchNorm()
            # batchnorm34_1_11_, output shape: {[8,28,28]}

            self.relu34_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_11_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_11_, output shape: {[512,28,28]}

            self.batchnorm35_1_11_ = gluon.nn.BatchNorm()
            # batchnorm35_1_11_, output shape: {[512,28,28]}

            self.conv33_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_12_, output shape: {[8,28,28]}

            self.batchnorm33_1_12_ = gluon.nn.BatchNorm()
            # batchnorm33_1_12_, output shape: {[8,28,28]}

            self.relu33_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_12_, output shape: {[8,28,28]}

            self.batchnorm34_1_12_ = gluon.nn.BatchNorm()
            # batchnorm34_1_12_, output shape: {[8,28,28]}

            self.relu34_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_12_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_12_, output shape: {[512,28,28]}

            self.batchnorm35_1_12_ = gluon.nn.BatchNorm()
            # batchnorm35_1_12_, output shape: {[512,28,28]}

            self.conv33_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_13_, output shape: {[8,28,28]}

            self.batchnorm33_1_13_ = gluon.nn.BatchNorm()
            # batchnorm33_1_13_, output shape: {[8,28,28]}

            self.relu33_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_13_, output shape: {[8,28,28]}

            self.batchnorm34_1_13_ = gluon.nn.BatchNorm()
            # batchnorm34_1_13_, output shape: {[8,28,28]}

            self.relu34_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_13_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_13_, output shape: {[512,28,28]}

            self.batchnorm35_1_13_ = gluon.nn.BatchNorm()
            # batchnorm35_1_13_, output shape: {[512,28,28]}

            self.conv33_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_14_, output shape: {[8,28,28]}

            self.batchnorm33_1_14_ = gluon.nn.BatchNorm()
            # batchnorm33_1_14_, output shape: {[8,28,28]}

            self.relu33_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_14_, output shape: {[8,28,28]}

            self.batchnorm34_1_14_ = gluon.nn.BatchNorm()
            # batchnorm34_1_14_, output shape: {[8,28,28]}

            self.relu34_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_14_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_14_, output shape: {[512,28,28]}

            self.batchnorm35_1_14_ = gluon.nn.BatchNorm()
            # batchnorm35_1_14_, output shape: {[512,28,28]}

            self.conv33_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_15_, output shape: {[8,28,28]}

            self.batchnorm33_1_15_ = gluon.nn.BatchNorm()
            # batchnorm33_1_15_, output shape: {[8,28,28]}

            self.relu33_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_15_, output shape: {[8,28,28]}

            self.batchnorm34_1_15_ = gluon.nn.BatchNorm()
            # batchnorm34_1_15_, output shape: {[8,28,28]}

            self.relu34_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_15_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_15_, output shape: {[512,28,28]}

            self.batchnorm35_1_15_ = gluon.nn.BatchNorm()
            # batchnorm35_1_15_, output shape: {[512,28,28]}

            self.conv33_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_16_, output shape: {[8,28,28]}

            self.batchnorm33_1_16_ = gluon.nn.BatchNorm()
            # batchnorm33_1_16_, output shape: {[8,28,28]}

            self.relu33_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_16_, output shape: {[8,28,28]}

            self.batchnorm34_1_16_ = gluon.nn.BatchNorm()
            # batchnorm34_1_16_, output shape: {[8,28,28]}

            self.relu34_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_16_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_16_, output shape: {[512,28,28]}

            self.batchnorm35_1_16_ = gluon.nn.BatchNorm()
            # batchnorm35_1_16_, output shape: {[512,28,28]}

            self.conv33_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_17_, output shape: {[8,28,28]}

            self.batchnorm33_1_17_ = gluon.nn.BatchNorm()
            # batchnorm33_1_17_, output shape: {[8,28,28]}

            self.relu33_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_17_, output shape: {[8,28,28]}

            self.batchnorm34_1_17_ = gluon.nn.BatchNorm()
            # batchnorm34_1_17_, output shape: {[8,28,28]}

            self.relu34_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_17_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_17_, output shape: {[512,28,28]}

            self.batchnorm35_1_17_ = gluon.nn.BatchNorm()
            # batchnorm35_1_17_, output shape: {[512,28,28]}

            self.conv33_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_18_, output shape: {[8,28,28]}

            self.batchnorm33_1_18_ = gluon.nn.BatchNorm()
            # batchnorm33_1_18_, output shape: {[8,28,28]}

            self.relu33_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_18_, output shape: {[8,28,28]}

            self.batchnorm34_1_18_ = gluon.nn.BatchNorm()
            # batchnorm34_1_18_, output shape: {[8,28,28]}

            self.relu34_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_18_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_18_, output shape: {[512,28,28]}

            self.batchnorm35_1_18_ = gluon.nn.BatchNorm()
            # batchnorm35_1_18_, output shape: {[512,28,28]}

            self.conv33_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_19_, output shape: {[8,28,28]}

            self.batchnorm33_1_19_ = gluon.nn.BatchNorm()
            # batchnorm33_1_19_, output shape: {[8,28,28]}

            self.relu33_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_19_, output shape: {[8,28,28]}

            self.batchnorm34_1_19_ = gluon.nn.BatchNorm()
            # batchnorm34_1_19_, output shape: {[8,28,28]}

            self.relu34_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_19_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_19_, output shape: {[512,28,28]}

            self.batchnorm35_1_19_ = gluon.nn.BatchNorm()
            # batchnorm35_1_19_, output shape: {[512,28,28]}

            self.conv33_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_20_, output shape: {[8,28,28]}

            self.batchnorm33_1_20_ = gluon.nn.BatchNorm()
            # batchnorm33_1_20_, output shape: {[8,28,28]}

            self.relu33_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_20_, output shape: {[8,28,28]}

            self.batchnorm34_1_20_ = gluon.nn.BatchNorm()
            # batchnorm34_1_20_, output shape: {[8,28,28]}

            self.relu34_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_20_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_20_, output shape: {[512,28,28]}

            self.batchnorm35_1_20_ = gluon.nn.BatchNorm()
            # batchnorm35_1_20_, output shape: {[512,28,28]}

            self.conv33_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_21_, output shape: {[8,28,28]}

            self.batchnorm33_1_21_ = gluon.nn.BatchNorm()
            # batchnorm33_1_21_, output shape: {[8,28,28]}

            self.relu33_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_21_, output shape: {[8,28,28]}

            self.batchnorm34_1_21_ = gluon.nn.BatchNorm()
            # batchnorm34_1_21_, output shape: {[8,28,28]}

            self.relu34_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_21_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_21_, output shape: {[512,28,28]}

            self.batchnorm35_1_21_ = gluon.nn.BatchNorm()
            # batchnorm35_1_21_, output shape: {[512,28,28]}

            self.conv33_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_22_, output shape: {[8,28,28]}

            self.batchnorm33_1_22_ = gluon.nn.BatchNorm()
            # batchnorm33_1_22_, output shape: {[8,28,28]}

            self.relu33_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_22_, output shape: {[8,28,28]}

            self.batchnorm34_1_22_ = gluon.nn.BatchNorm()
            # batchnorm34_1_22_, output shape: {[8,28,28]}

            self.relu34_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_22_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_22_, output shape: {[512,28,28]}

            self.batchnorm35_1_22_ = gluon.nn.BatchNorm()
            # batchnorm35_1_22_, output shape: {[512,28,28]}

            self.conv33_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_23_, output shape: {[8,28,28]}

            self.batchnorm33_1_23_ = gluon.nn.BatchNorm()
            # batchnorm33_1_23_, output shape: {[8,28,28]}

            self.relu33_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_23_, output shape: {[8,28,28]}

            self.batchnorm34_1_23_ = gluon.nn.BatchNorm()
            # batchnorm34_1_23_, output shape: {[8,28,28]}

            self.relu34_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_23_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_23_, output shape: {[512,28,28]}

            self.batchnorm35_1_23_ = gluon.nn.BatchNorm()
            # batchnorm35_1_23_, output shape: {[512,28,28]}

            self.conv33_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_24_, output shape: {[8,28,28]}

            self.batchnorm33_1_24_ = gluon.nn.BatchNorm()
            # batchnorm33_1_24_, output shape: {[8,28,28]}

            self.relu33_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_24_, output shape: {[8,28,28]}

            self.batchnorm34_1_24_ = gluon.nn.BatchNorm()
            # batchnorm34_1_24_, output shape: {[8,28,28]}

            self.relu34_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_24_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_24_, output shape: {[512,28,28]}

            self.batchnorm35_1_24_ = gluon.nn.BatchNorm()
            # batchnorm35_1_24_, output shape: {[512,28,28]}

            self.conv33_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_25_, output shape: {[8,28,28]}

            self.batchnorm33_1_25_ = gluon.nn.BatchNorm()
            # batchnorm33_1_25_, output shape: {[8,28,28]}

            self.relu33_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_25_, output shape: {[8,28,28]}

            self.batchnorm34_1_25_ = gluon.nn.BatchNorm()
            # batchnorm34_1_25_, output shape: {[8,28,28]}

            self.relu34_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_25_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_25_, output shape: {[512,28,28]}

            self.batchnorm35_1_25_ = gluon.nn.BatchNorm()
            # batchnorm35_1_25_, output shape: {[512,28,28]}

            self.conv33_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_26_, output shape: {[8,28,28]}

            self.batchnorm33_1_26_ = gluon.nn.BatchNorm()
            # batchnorm33_1_26_, output shape: {[8,28,28]}

            self.relu33_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_26_, output shape: {[8,28,28]}

            self.batchnorm34_1_26_ = gluon.nn.BatchNorm()
            # batchnorm34_1_26_, output shape: {[8,28,28]}

            self.relu34_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_26_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_26_, output shape: {[512,28,28]}

            self.batchnorm35_1_26_ = gluon.nn.BatchNorm()
            # batchnorm35_1_26_, output shape: {[512,28,28]}

            self.conv33_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_27_, output shape: {[8,28,28]}

            self.batchnorm33_1_27_ = gluon.nn.BatchNorm()
            # batchnorm33_1_27_, output shape: {[8,28,28]}

            self.relu33_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_27_, output shape: {[8,28,28]}

            self.batchnorm34_1_27_ = gluon.nn.BatchNorm()
            # batchnorm34_1_27_, output shape: {[8,28,28]}

            self.relu34_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_27_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_27_, output shape: {[512,28,28]}

            self.batchnorm35_1_27_ = gluon.nn.BatchNorm()
            # batchnorm35_1_27_, output shape: {[512,28,28]}

            self.conv33_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_28_, output shape: {[8,28,28]}

            self.batchnorm33_1_28_ = gluon.nn.BatchNorm()
            # batchnorm33_1_28_, output shape: {[8,28,28]}

            self.relu33_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_28_, output shape: {[8,28,28]}

            self.batchnorm34_1_28_ = gluon.nn.BatchNorm()
            # batchnorm34_1_28_, output shape: {[8,28,28]}

            self.relu34_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_28_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_28_, output shape: {[512,28,28]}

            self.batchnorm35_1_28_ = gluon.nn.BatchNorm()
            # batchnorm35_1_28_, output shape: {[512,28,28]}

            self.conv33_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_29_, output shape: {[8,28,28]}

            self.batchnorm33_1_29_ = gluon.nn.BatchNorm()
            # batchnorm33_1_29_, output shape: {[8,28,28]}

            self.relu33_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_29_, output shape: {[8,28,28]}

            self.batchnorm34_1_29_ = gluon.nn.BatchNorm()
            # batchnorm34_1_29_, output shape: {[8,28,28]}

            self.relu34_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_29_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_29_, output shape: {[512,28,28]}

            self.batchnorm35_1_29_ = gluon.nn.BatchNorm()
            # batchnorm35_1_29_, output shape: {[512,28,28]}

            self.conv33_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_30_, output shape: {[8,28,28]}

            self.batchnorm33_1_30_ = gluon.nn.BatchNorm()
            # batchnorm33_1_30_, output shape: {[8,28,28]}

            self.relu33_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_30_, output shape: {[8,28,28]}

            self.batchnorm34_1_30_ = gluon.nn.BatchNorm()
            # batchnorm34_1_30_, output shape: {[8,28,28]}

            self.relu34_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_30_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_30_, output shape: {[512,28,28]}

            self.batchnorm35_1_30_ = gluon.nn.BatchNorm()
            # batchnorm35_1_30_, output shape: {[512,28,28]}

            self.conv33_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_31_, output shape: {[8,28,28]}

            self.batchnorm33_1_31_ = gluon.nn.BatchNorm()
            # batchnorm33_1_31_, output shape: {[8,28,28]}

            self.relu33_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_31_, output shape: {[8,28,28]}

            self.batchnorm34_1_31_ = gluon.nn.BatchNorm()
            # batchnorm34_1_31_, output shape: {[8,28,28]}

            self.relu34_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_31_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_31_, output shape: {[512,28,28]}

            self.batchnorm35_1_31_ = gluon.nn.BatchNorm()
            # batchnorm35_1_31_, output shape: {[512,28,28]}

            self.conv33_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv33_1_32_, output shape: {[8,28,28]}

            self.batchnorm33_1_32_ = gluon.nn.BatchNorm()
            # batchnorm33_1_32_, output shape: {[8,28,28]}

            self.relu33_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv34_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv34_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv34_1_32_, output shape: {[8,28,28]}

            self.batchnorm34_1_32_ = gluon.nn.BatchNorm()
            # batchnorm34_1_32_, output shape: {[8,28,28]}

            self.relu34_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv35_1_32_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv35_1_32_, output shape: {[512,28,28]}

            self.batchnorm35_1_32_ = gluon.nn.BatchNorm()
            # batchnorm35_1_32_, output shape: {[512,28,28]}

            self.relu37_ = gluon.nn.Activation(activation='relu')
            self.conv39_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_1_, output shape: {[8,28,28]}

            self.batchnorm39_1_1_ = gluon.nn.BatchNorm()
            # batchnorm39_1_1_, output shape: {[8,28,28]}

            self.relu39_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_1_, output shape: {[8,28,28]}

            self.batchnorm40_1_1_ = gluon.nn.BatchNorm()
            # batchnorm40_1_1_, output shape: {[8,28,28]}

            self.relu40_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_1_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_1_, output shape: {[512,28,28]}

            self.batchnorm41_1_1_ = gluon.nn.BatchNorm()
            # batchnorm41_1_1_, output shape: {[512,28,28]}

            self.conv39_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_2_, output shape: {[8,28,28]}

            self.batchnorm39_1_2_ = gluon.nn.BatchNorm()
            # batchnorm39_1_2_, output shape: {[8,28,28]}

            self.relu39_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_2_, output shape: {[8,28,28]}

            self.batchnorm40_1_2_ = gluon.nn.BatchNorm()
            # batchnorm40_1_2_, output shape: {[8,28,28]}

            self.relu40_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_2_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_2_, output shape: {[512,28,28]}

            self.batchnorm41_1_2_ = gluon.nn.BatchNorm()
            # batchnorm41_1_2_, output shape: {[512,28,28]}

            self.conv39_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_3_, output shape: {[8,28,28]}

            self.batchnorm39_1_3_ = gluon.nn.BatchNorm()
            # batchnorm39_1_3_, output shape: {[8,28,28]}

            self.relu39_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_3_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_3_, output shape: {[8,28,28]}

            self.batchnorm40_1_3_ = gluon.nn.BatchNorm()
            # batchnorm40_1_3_, output shape: {[8,28,28]}

            self.relu40_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_3_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_3_, output shape: {[512,28,28]}

            self.batchnorm41_1_3_ = gluon.nn.BatchNorm()
            # batchnorm41_1_3_, output shape: {[512,28,28]}

            self.conv39_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_4_, output shape: {[8,28,28]}

            self.batchnorm39_1_4_ = gluon.nn.BatchNorm()
            # batchnorm39_1_4_, output shape: {[8,28,28]}

            self.relu39_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_4_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_4_, output shape: {[8,28,28]}

            self.batchnorm40_1_4_ = gluon.nn.BatchNorm()
            # batchnorm40_1_4_, output shape: {[8,28,28]}

            self.relu40_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_4_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_4_, output shape: {[512,28,28]}

            self.batchnorm41_1_4_ = gluon.nn.BatchNorm()
            # batchnorm41_1_4_, output shape: {[512,28,28]}

            self.conv39_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_5_, output shape: {[8,28,28]}

            self.batchnorm39_1_5_ = gluon.nn.BatchNorm()
            # batchnorm39_1_5_, output shape: {[8,28,28]}

            self.relu39_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_5_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_5_, output shape: {[8,28,28]}

            self.batchnorm40_1_5_ = gluon.nn.BatchNorm()
            # batchnorm40_1_5_, output shape: {[8,28,28]}

            self.relu40_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_5_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_5_, output shape: {[512,28,28]}

            self.batchnorm41_1_5_ = gluon.nn.BatchNorm()
            # batchnorm41_1_5_, output shape: {[512,28,28]}

            self.conv39_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_6_, output shape: {[8,28,28]}

            self.batchnorm39_1_6_ = gluon.nn.BatchNorm()
            # batchnorm39_1_6_, output shape: {[8,28,28]}

            self.relu39_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_6_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_6_, output shape: {[8,28,28]}

            self.batchnorm40_1_6_ = gluon.nn.BatchNorm()
            # batchnorm40_1_6_, output shape: {[8,28,28]}

            self.relu40_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_6_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_6_, output shape: {[512,28,28]}

            self.batchnorm41_1_6_ = gluon.nn.BatchNorm()
            # batchnorm41_1_6_, output shape: {[512,28,28]}

            self.conv39_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_7_, output shape: {[8,28,28]}

            self.batchnorm39_1_7_ = gluon.nn.BatchNorm()
            # batchnorm39_1_7_, output shape: {[8,28,28]}

            self.relu39_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_7_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_7_, output shape: {[8,28,28]}

            self.batchnorm40_1_7_ = gluon.nn.BatchNorm()
            # batchnorm40_1_7_, output shape: {[8,28,28]}

            self.relu40_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_7_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_7_, output shape: {[512,28,28]}

            self.batchnorm41_1_7_ = gluon.nn.BatchNorm()
            # batchnorm41_1_7_, output shape: {[512,28,28]}

            self.conv39_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_8_, output shape: {[8,28,28]}

            self.batchnorm39_1_8_ = gluon.nn.BatchNorm()
            # batchnorm39_1_8_, output shape: {[8,28,28]}

            self.relu39_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_8_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_8_, output shape: {[8,28,28]}

            self.batchnorm40_1_8_ = gluon.nn.BatchNorm()
            # batchnorm40_1_8_, output shape: {[8,28,28]}

            self.relu40_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_8_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_8_, output shape: {[512,28,28]}

            self.batchnorm41_1_8_ = gluon.nn.BatchNorm()
            # batchnorm41_1_8_, output shape: {[512,28,28]}

            self.conv39_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_9_, output shape: {[8,28,28]}

            self.batchnorm39_1_9_ = gluon.nn.BatchNorm()
            # batchnorm39_1_9_, output shape: {[8,28,28]}

            self.relu39_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_9_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_9_, output shape: {[8,28,28]}

            self.batchnorm40_1_9_ = gluon.nn.BatchNorm()
            # batchnorm40_1_9_, output shape: {[8,28,28]}

            self.relu40_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_9_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_9_, output shape: {[512,28,28]}

            self.batchnorm41_1_9_ = gluon.nn.BatchNorm()
            # batchnorm41_1_9_, output shape: {[512,28,28]}

            self.conv39_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_10_, output shape: {[8,28,28]}

            self.batchnorm39_1_10_ = gluon.nn.BatchNorm()
            # batchnorm39_1_10_, output shape: {[8,28,28]}

            self.relu39_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_10_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_10_, output shape: {[8,28,28]}

            self.batchnorm40_1_10_ = gluon.nn.BatchNorm()
            # batchnorm40_1_10_, output shape: {[8,28,28]}

            self.relu40_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_10_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_10_, output shape: {[512,28,28]}

            self.batchnorm41_1_10_ = gluon.nn.BatchNorm()
            # batchnorm41_1_10_, output shape: {[512,28,28]}

            self.conv39_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_11_, output shape: {[8,28,28]}

            self.batchnorm39_1_11_ = gluon.nn.BatchNorm()
            # batchnorm39_1_11_, output shape: {[8,28,28]}

            self.relu39_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_11_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_11_, output shape: {[8,28,28]}

            self.batchnorm40_1_11_ = gluon.nn.BatchNorm()
            # batchnorm40_1_11_, output shape: {[8,28,28]}

            self.relu40_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_11_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_11_, output shape: {[512,28,28]}

            self.batchnorm41_1_11_ = gluon.nn.BatchNorm()
            # batchnorm41_1_11_, output shape: {[512,28,28]}

            self.conv39_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_12_, output shape: {[8,28,28]}

            self.batchnorm39_1_12_ = gluon.nn.BatchNorm()
            # batchnorm39_1_12_, output shape: {[8,28,28]}

            self.relu39_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_12_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_12_, output shape: {[8,28,28]}

            self.batchnorm40_1_12_ = gluon.nn.BatchNorm()
            # batchnorm40_1_12_, output shape: {[8,28,28]}

            self.relu40_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_12_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_12_, output shape: {[512,28,28]}

            self.batchnorm41_1_12_ = gluon.nn.BatchNorm()
            # batchnorm41_1_12_, output shape: {[512,28,28]}

            self.conv39_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_13_, output shape: {[8,28,28]}

            self.batchnorm39_1_13_ = gluon.nn.BatchNorm()
            # batchnorm39_1_13_, output shape: {[8,28,28]}

            self.relu39_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_13_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_13_, output shape: {[8,28,28]}

            self.batchnorm40_1_13_ = gluon.nn.BatchNorm()
            # batchnorm40_1_13_, output shape: {[8,28,28]}

            self.relu40_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_13_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_13_, output shape: {[512,28,28]}

            self.batchnorm41_1_13_ = gluon.nn.BatchNorm()
            # batchnorm41_1_13_, output shape: {[512,28,28]}

            self.conv39_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_14_, output shape: {[8,28,28]}

            self.batchnorm39_1_14_ = gluon.nn.BatchNorm()
            # batchnorm39_1_14_, output shape: {[8,28,28]}

            self.relu39_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_14_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_14_, output shape: {[8,28,28]}

            self.batchnorm40_1_14_ = gluon.nn.BatchNorm()
            # batchnorm40_1_14_, output shape: {[8,28,28]}

            self.relu40_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_14_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_14_, output shape: {[512,28,28]}

            self.batchnorm41_1_14_ = gluon.nn.BatchNorm()
            # batchnorm41_1_14_, output shape: {[512,28,28]}

            self.conv39_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_15_, output shape: {[8,28,28]}

            self.batchnorm39_1_15_ = gluon.nn.BatchNorm()
            # batchnorm39_1_15_, output shape: {[8,28,28]}

            self.relu39_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_15_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_15_, output shape: {[8,28,28]}

            self.batchnorm40_1_15_ = gluon.nn.BatchNorm()
            # batchnorm40_1_15_, output shape: {[8,28,28]}

            self.relu40_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_15_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_15_, output shape: {[512,28,28]}

            self.batchnorm41_1_15_ = gluon.nn.BatchNorm()
            # batchnorm41_1_15_, output shape: {[512,28,28]}

            self.conv39_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_16_, output shape: {[8,28,28]}

            self.batchnorm39_1_16_ = gluon.nn.BatchNorm()
            # batchnorm39_1_16_, output shape: {[8,28,28]}

            self.relu39_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_16_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_16_, output shape: {[8,28,28]}

            self.batchnorm40_1_16_ = gluon.nn.BatchNorm()
            # batchnorm40_1_16_, output shape: {[8,28,28]}

            self.relu40_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_16_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_16_, output shape: {[512,28,28]}

            self.batchnorm41_1_16_ = gluon.nn.BatchNorm()
            # batchnorm41_1_16_, output shape: {[512,28,28]}

            self.conv39_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_17_, output shape: {[8,28,28]}

            self.batchnorm39_1_17_ = gluon.nn.BatchNorm()
            # batchnorm39_1_17_, output shape: {[8,28,28]}

            self.relu39_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_17_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_17_, output shape: {[8,28,28]}

            self.batchnorm40_1_17_ = gluon.nn.BatchNorm()
            # batchnorm40_1_17_, output shape: {[8,28,28]}

            self.relu40_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_17_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_17_, output shape: {[512,28,28]}

            self.batchnorm41_1_17_ = gluon.nn.BatchNorm()
            # batchnorm41_1_17_, output shape: {[512,28,28]}

            self.conv39_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_18_, output shape: {[8,28,28]}

            self.batchnorm39_1_18_ = gluon.nn.BatchNorm()
            # batchnorm39_1_18_, output shape: {[8,28,28]}

            self.relu39_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_18_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_18_, output shape: {[8,28,28]}

            self.batchnorm40_1_18_ = gluon.nn.BatchNorm()
            # batchnorm40_1_18_, output shape: {[8,28,28]}

            self.relu40_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_18_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_18_, output shape: {[512,28,28]}

            self.batchnorm41_1_18_ = gluon.nn.BatchNorm()
            # batchnorm41_1_18_, output shape: {[512,28,28]}

            self.conv39_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_19_, output shape: {[8,28,28]}

            self.batchnorm39_1_19_ = gluon.nn.BatchNorm()
            # batchnorm39_1_19_, output shape: {[8,28,28]}

            self.relu39_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_19_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_19_, output shape: {[8,28,28]}

            self.batchnorm40_1_19_ = gluon.nn.BatchNorm()
            # batchnorm40_1_19_, output shape: {[8,28,28]}

            self.relu40_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_19_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_19_, output shape: {[512,28,28]}

            self.batchnorm41_1_19_ = gluon.nn.BatchNorm()
            # batchnorm41_1_19_, output shape: {[512,28,28]}

            self.conv39_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_20_, output shape: {[8,28,28]}

            self.batchnorm39_1_20_ = gluon.nn.BatchNorm()
            # batchnorm39_1_20_, output shape: {[8,28,28]}

            self.relu39_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_20_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_20_, output shape: {[8,28,28]}

            self.batchnorm40_1_20_ = gluon.nn.BatchNorm()
            # batchnorm40_1_20_, output shape: {[8,28,28]}

            self.relu40_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_20_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_20_, output shape: {[512,28,28]}

            self.batchnorm41_1_20_ = gluon.nn.BatchNorm()
            # batchnorm41_1_20_, output shape: {[512,28,28]}

            self.conv39_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_21_, output shape: {[8,28,28]}

            self.batchnorm39_1_21_ = gluon.nn.BatchNorm()
            # batchnorm39_1_21_, output shape: {[8,28,28]}

            self.relu39_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_21_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_21_, output shape: {[8,28,28]}

            self.batchnorm40_1_21_ = gluon.nn.BatchNorm()
            # batchnorm40_1_21_, output shape: {[8,28,28]}

            self.relu40_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_21_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_21_, output shape: {[512,28,28]}

            self.batchnorm41_1_21_ = gluon.nn.BatchNorm()
            # batchnorm41_1_21_, output shape: {[512,28,28]}

            self.conv39_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_22_, output shape: {[8,28,28]}

            self.batchnorm39_1_22_ = gluon.nn.BatchNorm()
            # batchnorm39_1_22_, output shape: {[8,28,28]}

            self.relu39_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_22_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_22_, output shape: {[8,28,28]}

            self.batchnorm40_1_22_ = gluon.nn.BatchNorm()
            # batchnorm40_1_22_, output shape: {[8,28,28]}

            self.relu40_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_22_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_22_, output shape: {[512,28,28]}

            self.batchnorm41_1_22_ = gluon.nn.BatchNorm()
            # batchnorm41_1_22_, output shape: {[512,28,28]}

            self.conv39_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_23_, output shape: {[8,28,28]}

            self.batchnorm39_1_23_ = gluon.nn.BatchNorm()
            # batchnorm39_1_23_, output shape: {[8,28,28]}

            self.relu39_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_23_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_23_, output shape: {[8,28,28]}

            self.batchnorm40_1_23_ = gluon.nn.BatchNorm()
            # batchnorm40_1_23_, output shape: {[8,28,28]}

            self.relu40_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_23_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_23_, output shape: {[512,28,28]}

            self.batchnorm41_1_23_ = gluon.nn.BatchNorm()
            # batchnorm41_1_23_, output shape: {[512,28,28]}

            self.conv39_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_24_, output shape: {[8,28,28]}

            self.batchnorm39_1_24_ = gluon.nn.BatchNorm()
            # batchnorm39_1_24_, output shape: {[8,28,28]}

            self.relu39_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_24_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_24_, output shape: {[8,28,28]}

            self.batchnorm40_1_24_ = gluon.nn.BatchNorm()
            # batchnorm40_1_24_, output shape: {[8,28,28]}

            self.relu40_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_24_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_24_, output shape: {[512,28,28]}

            self.batchnorm41_1_24_ = gluon.nn.BatchNorm()
            # batchnorm41_1_24_, output shape: {[512,28,28]}

            self.conv39_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_25_, output shape: {[8,28,28]}

            self.batchnorm39_1_25_ = gluon.nn.BatchNorm()
            # batchnorm39_1_25_, output shape: {[8,28,28]}

            self.relu39_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_25_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_25_, output shape: {[8,28,28]}

            self.batchnorm40_1_25_ = gluon.nn.BatchNorm()
            # batchnorm40_1_25_, output shape: {[8,28,28]}

            self.relu40_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_25_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_25_, output shape: {[512,28,28]}

            self.batchnorm41_1_25_ = gluon.nn.BatchNorm()
            # batchnorm41_1_25_, output shape: {[512,28,28]}

            self.conv39_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_26_, output shape: {[8,28,28]}

            self.batchnorm39_1_26_ = gluon.nn.BatchNorm()
            # batchnorm39_1_26_, output shape: {[8,28,28]}

            self.relu39_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_26_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_26_, output shape: {[8,28,28]}

            self.batchnorm40_1_26_ = gluon.nn.BatchNorm()
            # batchnorm40_1_26_, output shape: {[8,28,28]}

            self.relu40_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_26_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_26_, output shape: {[512,28,28]}

            self.batchnorm41_1_26_ = gluon.nn.BatchNorm()
            # batchnorm41_1_26_, output shape: {[512,28,28]}

            self.conv39_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_27_, output shape: {[8,28,28]}

            self.batchnorm39_1_27_ = gluon.nn.BatchNorm()
            # batchnorm39_1_27_, output shape: {[8,28,28]}

            self.relu39_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_27_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_27_, output shape: {[8,28,28]}

            self.batchnorm40_1_27_ = gluon.nn.BatchNorm()
            # batchnorm40_1_27_, output shape: {[8,28,28]}

            self.relu40_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_27_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_27_, output shape: {[512,28,28]}

            self.batchnorm41_1_27_ = gluon.nn.BatchNorm()
            # batchnorm41_1_27_, output shape: {[512,28,28]}

            self.conv39_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_28_, output shape: {[8,28,28]}

            self.batchnorm39_1_28_ = gluon.nn.BatchNorm()
            # batchnorm39_1_28_, output shape: {[8,28,28]}

            self.relu39_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_28_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_28_, output shape: {[8,28,28]}

            self.batchnorm40_1_28_ = gluon.nn.BatchNorm()
            # batchnorm40_1_28_, output shape: {[8,28,28]}

            self.relu40_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_28_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_28_, output shape: {[512,28,28]}

            self.batchnorm41_1_28_ = gluon.nn.BatchNorm()
            # batchnorm41_1_28_, output shape: {[512,28,28]}

            self.conv39_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_29_, output shape: {[8,28,28]}

            self.batchnorm39_1_29_ = gluon.nn.BatchNorm()
            # batchnorm39_1_29_, output shape: {[8,28,28]}

            self.relu39_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_29_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_29_, output shape: {[8,28,28]}

            self.batchnorm40_1_29_ = gluon.nn.BatchNorm()
            # batchnorm40_1_29_, output shape: {[8,28,28]}

            self.relu40_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_29_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_29_, output shape: {[512,28,28]}

            self.batchnorm41_1_29_ = gluon.nn.BatchNorm()
            # batchnorm41_1_29_, output shape: {[512,28,28]}

            self.conv39_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_30_, output shape: {[8,28,28]}

            self.batchnorm39_1_30_ = gluon.nn.BatchNorm()
            # batchnorm39_1_30_, output shape: {[8,28,28]}

            self.relu39_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_30_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_30_, output shape: {[8,28,28]}

            self.batchnorm40_1_30_ = gluon.nn.BatchNorm()
            # batchnorm40_1_30_, output shape: {[8,28,28]}

            self.relu40_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_30_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_30_, output shape: {[512,28,28]}

            self.batchnorm41_1_30_ = gluon.nn.BatchNorm()
            # batchnorm41_1_30_, output shape: {[512,28,28]}

            self.conv39_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_31_, output shape: {[8,28,28]}

            self.batchnorm39_1_31_ = gluon.nn.BatchNorm()
            # batchnorm39_1_31_, output shape: {[8,28,28]}

            self.relu39_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_31_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_31_, output shape: {[8,28,28]}

            self.batchnorm40_1_31_ = gluon.nn.BatchNorm()
            # batchnorm40_1_31_, output shape: {[8,28,28]}

            self.relu40_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_31_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_31_, output shape: {[512,28,28]}

            self.batchnorm41_1_31_ = gluon.nn.BatchNorm()
            # batchnorm41_1_31_, output shape: {[512,28,28]}

            self.conv39_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv39_1_32_, output shape: {[8,28,28]}

            self.batchnorm39_1_32_ = gluon.nn.BatchNorm()
            # batchnorm39_1_32_, output shape: {[8,28,28]}

            self.relu39_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv40_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv40_1_32_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv40_1_32_, output shape: {[8,28,28]}

            self.batchnorm40_1_32_ = gluon.nn.BatchNorm()
            # batchnorm40_1_32_, output shape: {[8,28,28]}

            self.relu40_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv41_1_32_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv41_1_32_, output shape: {[512,28,28]}

            self.batchnorm41_1_32_ = gluon.nn.BatchNorm()
            # batchnorm41_1_32_, output shape: {[512,28,28]}

            self.relu43_ = gluon.nn.Activation(activation='relu')
            self.conv45_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_1_, output shape: {[16,28,28]}

            self.batchnorm45_1_1_ = gluon.nn.BatchNorm()
            # batchnorm45_1_1_, output shape: {[16,28,28]}

            self.relu45_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_1_, output shape: {[16,14,14]}

            self.batchnorm46_1_1_ = gluon.nn.BatchNorm()
            # batchnorm46_1_1_, output shape: {[16,14,14]}

            self.relu46_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_1_, output shape: {[1024,14,14]}

            self.batchnorm47_1_1_ = gluon.nn.BatchNorm()
            # batchnorm47_1_1_, output shape: {[1024,14,14]}

            self.conv45_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_2_, output shape: {[16,28,28]}

            self.batchnorm45_1_2_ = gluon.nn.BatchNorm()
            # batchnorm45_1_2_, output shape: {[16,28,28]}

            self.relu45_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_2_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_2_, output shape: {[16,14,14]}

            self.batchnorm46_1_2_ = gluon.nn.BatchNorm()
            # batchnorm46_1_2_, output shape: {[16,14,14]}

            self.relu46_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_2_, output shape: {[1024,14,14]}

            self.batchnorm47_1_2_ = gluon.nn.BatchNorm()
            # batchnorm47_1_2_, output shape: {[1024,14,14]}

            self.conv45_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_3_, output shape: {[16,28,28]}

            self.batchnorm45_1_3_ = gluon.nn.BatchNorm()
            # batchnorm45_1_3_, output shape: {[16,28,28]}

            self.relu45_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_3_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_3_, output shape: {[16,14,14]}

            self.batchnorm46_1_3_ = gluon.nn.BatchNorm()
            # batchnorm46_1_3_, output shape: {[16,14,14]}

            self.relu46_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_3_, output shape: {[1024,14,14]}

            self.batchnorm47_1_3_ = gluon.nn.BatchNorm()
            # batchnorm47_1_3_, output shape: {[1024,14,14]}

            self.conv45_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_4_, output shape: {[16,28,28]}

            self.batchnorm45_1_4_ = gluon.nn.BatchNorm()
            # batchnorm45_1_4_, output shape: {[16,28,28]}

            self.relu45_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_4_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_4_, output shape: {[16,14,14]}

            self.batchnorm46_1_4_ = gluon.nn.BatchNorm()
            # batchnorm46_1_4_, output shape: {[16,14,14]}

            self.relu46_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_4_, output shape: {[1024,14,14]}

            self.batchnorm47_1_4_ = gluon.nn.BatchNorm()
            # batchnorm47_1_4_, output shape: {[1024,14,14]}

            self.conv45_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_5_, output shape: {[16,28,28]}

            self.batchnorm45_1_5_ = gluon.nn.BatchNorm()
            # batchnorm45_1_5_, output shape: {[16,28,28]}

            self.relu45_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_5_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_5_, output shape: {[16,14,14]}

            self.batchnorm46_1_5_ = gluon.nn.BatchNorm()
            # batchnorm46_1_5_, output shape: {[16,14,14]}

            self.relu46_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_5_, output shape: {[1024,14,14]}

            self.batchnorm47_1_5_ = gluon.nn.BatchNorm()
            # batchnorm47_1_5_, output shape: {[1024,14,14]}

            self.conv45_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_6_, output shape: {[16,28,28]}

            self.batchnorm45_1_6_ = gluon.nn.BatchNorm()
            # batchnorm45_1_6_, output shape: {[16,28,28]}

            self.relu45_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_6_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_6_, output shape: {[16,14,14]}

            self.batchnorm46_1_6_ = gluon.nn.BatchNorm()
            # batchnorm46_1_6_, output shape: {[16,14,14]}

            self.relu46_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_6_, output shape: {[1024,14,14]}

            self.batchnorm47_1_6_ = gluon.nn.BatchNorm()
            # batchnorm47_1_6_, output shape: {[1024,14,14]}

            self.conv45_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_7_, output shape: {[16,28,28]}

            self.batchnorm45_1_7_ = gluon.nn.BatchNorm()
            # batchnorm45_1_7_, output shape: {[16,28,28]}

            self.relu45_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_7_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_7_, output shape: {[16,14,14]}

            self.batchnorm46_1_7_ = gluon.nn.BatchNorm()
            # batchnorm46_1_7_, output shape: {[16,14,14]}

            self.relu46_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_7_, output shape: {[1024,14,14]}

            self.batchnorm47_1_7_ = gluon.nn.BatchNorm()
            # batchnorm47_1_7_, output shape: {[1024,14,14]}

            self.conv45_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_8_, output shape: {[16,28,28]}

            self.batchnorm45_1_8_ = gluon.nn.BatchNorm()
            # batchnorm45_1_8_, output shape: {[16,28,28]}

            self.relu45_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_8_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_8_, output shape: {[16,14,14]}

            self.batchnorm46_1_8_ = gluon.nn.BatchNorm()
            # batchnorm46_1_8_, output shape: {[16,14,14]}

            self.relu46_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_8_, output shape: {[1024,14,14]}

            self.batchnorm47_1_8_ = gluon.nn.BatchNorm()
            # batchnorm47_1_8_, output shape: {[1024,14,14]}

            self.conv45_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_9_, output shape: {[16,28,28]}

            self.batchnorm45_1_9_ = gluon.nn.BatchNorm()
            # batchnorm45_1_9_, output shape: {[16,28,28]}

            self.relu45_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_9_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_9_, output shape: {[16,14,14]}

            self.batchnorm46_1_9_ = gluon.nn.BatchNorm()
            # batchnorm46_1_9_, output shape: {[16,14,14]}

            self.relu46_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_9_, output shape: {[1024,14,14]}

            self.batchnorm47_1_9_ = gluon.nn.BatchNorm()
            # batchnorm47_1_9_, output shape: {[1024,14,14]}

            self.conv45_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_10_, output shape: {[16,28,28]}

            self.batchnorm45_1_10_ = gluon.nn.BatchNorm()
            # batchnorm45_1_10_, output shape: {[16,28,28]}

            self.relu45_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_10_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_10_, output shape: {[16,14,14]}

            self.batchnorm46_1_10_ = gluon.nn.BatchNorm()
            # batchnorm46_1_10_, output shape: {[16,14,14]}

            self.relu46_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_10_, output shape: {[1024,14,14]}

            self.batchnorm47_1_10_ = gluon.nn.BatchNorm()
            # batchnorm47_1_10_, output shape: {[1024,14,14]}

            self.conv45_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_11_, output shape: {[16,28,28]}

            self.batchnorm45_1_11_ = gluon.nn.BatchNorm()
            # batchnorm45_1_11_, output shape: {[16,28,28]}

            self.relu45_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_11_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_11_, output shape: {[16,14,14]}

            self.batchnorm46_1_11_ = gluon.nn.BatchNorm()
            # batchnorm46_1_11_, output shape: {[16,14,14]}

            self.relu46_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_11_, output shape: {[1024,14,14]}

            self.batchnorm47_1_11_ = gluon.nn.BatchNorm()
            # batchnorm47_1_11_, output shape: {[1024,14,14]}

            self.conv45_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_12_, output shape: {[16,28,28]}

            self.batchnorm45_1_12_ = gluon.nn.BatchNorm()
            # batchnorm45_1_12_, output shape: {[16,28,28]}

            self.relu45_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_12_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_12_, output shape: {[16,14,14]}

            self.batchnorm46_1_12_ = gluon.nn.BatchNorm()
            # batchnorm46_1_12_, output shape: {[16,14,14]}

            self.relu46_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_12_, output shape: {[1024,14,14]}

            self.batchnorm47_1_12_ = gluon.nn.BatchNorm()
            # batchnorm47_1_12_, output shape: {[1024,14,14]}

            self.conv45_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_13_, output shape: {[16,28,28]}

            self.batchnorm45_1_13_ = gluon.nn.BatchNorm()
            # batchnorm45_1_13_, output shape: {[16,28,28]}

            self.relu45_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_13_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_13_, output shape: {[16,14,14]}

            self.batchnorm46_1_13_ = gluon.nn.BatchNorm()
            # batchnorm46_1_13_, output shape: {[16,14,14]}

            self.relu46_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_13_, output shape: {[1024,14,14]}

            self.batchnorm47_1_13_ = gluon.nn.BatchNorm()
            # batchnorm47_1_13_, output shape: {[1024,14,14]}

            self.conv45_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_14_, output shape: {[16,28,28]}

            self.batchnorm45_1_14_ = gluon.nn.BatchNorm()
            # batchnorm45_1_14_, output shape: {[16,28,28]}

            self.relu45_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_14_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_14_, output shape: {[16,14,14]}

            self.batchnorm46_1_14_ = gluon.nn.BatchNorm()
            # batchnorm46_1_14_, output shape: {[16,14,14]}

            self.relu46_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_14_, output shape: {[1024,14,14]}

            self.batchnorm47_1_14_ = gluon.nn.BatchNorm()
            # batchnorm47_1_14_, output shape: {[1024,14,14]}

            self.conv45_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_15_, output shape: {[16,28,28]}

            self.batchnorm45_1_15_ = gluon.nn.BatchNorm()
            # batchnorm45_1_15_, output shape: {[16,28,28]}

            self.relu45_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_15_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_15_, output shape: {[16,14,14]}

            self.batchnorm46_1_15_ = gluon.nn.BatchNorm()
            # batchnorm46_1_15_, output shape: {[16,14,14]}

            self.relu46_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_15_, output shape: {[1024,14,14]}

            self.batchnorm47_1_15_ = gluon.nn.BatchNorm()
            # batchnorm47_1_15_, output shape: {[1024,14,14]}

            self.conv45_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_16_, output shape: {[16,28,28]}

            self.batchnorm45_1_16_ = gluon.nn.BatchNorm()
            # batchnorm45_1_16_, output shape: {[16,28,28]}

            self.relu45_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_16_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_16_, output shape: {[16,14,14]}

            self.batchnorm46_1_16_ = gluon.nn.BatchNorm()
            # batchnorm46_1_16_, output shape: {[16,14,14]}

            self.relu46_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_16_, output shape: {[1024,14,14]}

            self.batchnorm47_1_16_ = gluon.nn.BatchNorm()
            # batchnorm47_1_16_, output shape: {[1024,14,14]}

            self.conv45_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_17_, output shape: {[16,28,28]}

            self.batchnorm45_1_17_ = gluon.nn.BatchNorm()
            # batchnorm45_1_17_, output shape: {[16,28,28]}

            self.relu45_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_17_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_17_, output shape: {[16,14,14]}

            self.batchnorm46_1_17_ = gluon.nn.BatchNorm()
            # batchnorm46_1_17_, output shape: {[16,14,14]}

            self.relu46_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_17_, output shape: {[1024,14,14]}

            self.batchnorm47_1_17_ = gluon.nn.BatchNorm()
            # batchnorm47_1_17_, output shape: {[1024,14,14]}

            self.conv45_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_18_, output shape: {[16,28,28]}

            self.batchnorm45_1_18_ = gluon.nn.BatchNorm()
            # batchnorm45_1_18_, output shape: {[16,28,28]}

            self.relu45_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_18_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_18_, output shape: {[16,14,14]}

            self.batchnorm46_1_18_ = gluon.nn.BatchNorm()
            # batchnorm46_1_18_, output shape: {[16,14,14]}

            self.relu46_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_18_, output shape: {[1024,14,14]}

            self.batchnorm47_1_18_ = gluon.nn.BatchNorm()
            # batchnorm47_1_18_, output shape: {[1024,14,14]}

            self.conv45_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_19_, output shape: {[16,28,28]}

            self.batchnorm45_1_19_ = gluon.nn.BatchNorm()
            # batchnorm45_1_19_, output shape: {[16,28,28]}

            self.relu45_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_19_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_19_, output shape: {[16,14,14]}

            self.batchnorm46_1_19_ = gluon.nn.BatchNorm()
            # batchnorm46_1_19_, output shape: {[16,14,14]}

            self.relu46_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_19_, output shape: {[1024,14,14]}

            self.batchnorm47_1_19_ = gluon.nn.BatchNorm()
            # batchnorm47_1_19_, output shape: {[1024,14,14]}

            self.conv45_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_20_, output shape: {[16,28,28]}

            self.batchnorm45_1_20_ = gluon.nn.BatchNorm()
            # batchnorm45_1_20_, output shape: {[16,28,28]}

            self.relu45_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_20_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_20_, output shape: {[16,14,14]}

            self.batchnorm46_1_20_ = gluon.nn.BatchNorm()
            # batchnorm46_1_20_, output shape: {[16,14,14]}

            self.relu46_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_20_, output shape: {[1024,14,14]}

            self.batchnorm47_1_20_ = gluon.nn.BatchNorm()
            # batchnorm47_1_20_, output shape: {[1024,14,14]}

            self.conv45_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_21_, output shape: {[16,28,28]}

            self.batchnorm45_1_21_ = gluon.nn.BatchNorm()
            # batchnorm45_1_21_, output shape: {[16,28,28]}

            self.relu45_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_21_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_21_, output shape: {[16,14,14]}

            self.batchnorm46_1_21_ = gluon.nn.BatchNorm()
            # batchnorm46_1_21_, output shape: {[16,14,14]}

            self.relu46_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_21_, output shape: {[1024,14,14]}

            self.batchnorm47_1_21_ = gluon.nn.BatchNorm()
            # batchnorm47_1_21_, output shape: {[1024,14,14]}

            self.conv45_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_22_, output shape: {[16,28,28]}

            self.batchnorm45_1_22_ = gluon.nn.BatchNorm()
            # batchnorm45_1_22_, output shape: {[16,28,28]}

            self.relu45_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_22_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_22_, output shape: {[16,14,14]}

            self.batchnorm46_1_22_ = gluon.nn.BatchNorm()
            # batchnorm46_1_22_, output shape: {[16,14,14]}

            self.relu46_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_22_, output shape: {[1024,14,14]}

            self.batchnorm47_1_22_ = gluon.nn.BatchNorm()
            # batchnorm47_1_22_, output shape: {[1024,14,14]}

            self.conv45_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_23_, output shape: {[16,28,28]}

            self.batchnorm45_1_23_ = gluon.nn.BatchNorm()
            # batchnorm45_1_23_, output shape: {[16,28,28]}

            self.relu45_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_23_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_23_, output shape: {[16,14,14]}

            self.batchnorm46_1_23_ = gluon.nn.BatchNorm()
            # batchnorm46_1_23_, output shape: {[16,14,14]}

            self.relu46_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_23_, output shape: {[1024,14,14]}

            self.batchnorm47_1_23_ = gluon.nn.BatchNorm()
            # batchnorm47_1_23_, output shape: {[1024,14,14]}

            self.conv45_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_24_, output shape: {[16,28,28]}

            self.batchnorm45_1_24_ = gluon.nn.BatchNorm()
            # batchnorm45_1_24_, output shape: {[16,28,28]}

            self.relu45_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_24_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_24_, output shape: {[16,14,14]}

            self.batchnorm46_1_24_ = gluon.nn.BatchNorm()
            # batchnorm46_1_24_, output shape: {[16,14,14]}

            self.relu46_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_24_, output shape: {[1024,14,14]}

            self.batchnorm47_1_24_ = gluon.nn.BatchNorm()
            # batchnorm47_1_24_, output shape: {[1024,14,14]}

            self.conv45_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_25_, output shape: {[16,28,28]}

            self.batchnorm45_1_25_ = gluon.nn.BatchNorm()
            # batchnorm45_1_25_, output shape: {[16,28,28]}

            self.relu45_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_25_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_25_, output shape: {[16,14,14]}

            self.batchnorm46_1_25_ = gluon.nn.BatchNorm()
            # batchnorm46_1_25_, output shape: {[16,14,14]}

            self.relu46_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_25_, output shape: {[1024,14,14]}

            self.batchnorm47_1_25_ = gluon.nn.BatchNorm()
            # batchnorm47_1_25_, output shape: {[1024,14,14]}

            self.conv45_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_26_, output shape: {[16,28,28]}

            self.batchnorm45_1_26_ = gluon.nn.BatchNorm()
            # batchnorm45_1_26_, output shape: {[16,28,28]}

            self.relu45_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_26_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_26_, output shape: {[16,14,14]}

            self.batchnorm46_1_26_ = gluon.nn.BatchNorm()
            # batchnorm46_1_26_, output shape: {[16,14,14]}

            self.relu46_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_26_, output shape: {[1024,14,14]}

            self.batchnorm47_1_26_ = gluon.nn.BatchNorm()
            # batchnorm47_1_26_, output shape: {[1024,14,14]}

            self.conv45_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_27_, output shape: {[16,28,28]}

            self.batchnorm45_1_27_ = gluon.nn.BatchNorm()
            # batchnorm45_1_27_, output shape: {[16,28,28]}

            self.relu45_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_27_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_27_, output shape: {[16,14,14]}

            self.batchnorm46_1_27_ = gluon.nn.BatchNorm()
            # batchnorm46_1_27_, output shape: {[16,14,14]}

            self.relu46_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_27_, output shape: {[1024,14,14]}

            self.batchnorm47_1_27_ = gluon.nn.BatchNorm()
            # batchnorm47_1_27_, output shape: {[1024,14,14]}

            self.conv45_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_28_, output shape: {[16,28,28]}

            self.batchnorm45_1_28_ = gluon.nn.BatchNorm()
            # batchnorm45_1_28_, output shape: {[16,28,28]}

            self.relu45_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_28_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_28_, output shape: {[16,14,14]}

            self.batchnorm46_1_28_ = gluon.nn.BatchNorm()
            # batchnorm46_1_28_, output shape: {[16,14,14]}

            self.relu46_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_28_, output shape: {[1024,14,14]}

            self.batchnorm47_1_28_ = gluon.nn.BatchNorm()
            # batchnorm47_1_28_, output shape: {[1024,14,14]}

            self.conv45_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_29_, output shape: {[16,28,28]}

            self.batchnorm45_1_29_ = gluon.nn.BatchNorm()
            # batchnorm45_1_29_, output shape: {[16,28,28]}

            self.relu45_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_29_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_29_, output shape: {[16,14,14]}

            self.batchnorm46_1_29_ = gluon.nn.BatchNorm()
            # batchnorm46_1_29_, output shape: {[16,14,14]}

            self.relu46_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_29_, output shape: {[1024,14,14]}

            self.batchnorm47_1_29_ = gluon.nn.BatchNorm()
            # batchnorm47_1_29_, output shape: {[1024,14,14]}

            self.conv45_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_30_, output shape: {[16,28,28]}

            self.batchnorm45_1_30_ = gluon.nn.BatchNorm()
            # batchnorm45_1_30_, output shape: {[16,28,28]}

            self.relu45_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_30_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_30_, output shape: {[16,14,14]}

            self.batchnorm46_1_30_ = gluon.nn.BatchNorm()
            # batchnorm46_1_30_, output shape: {[16,14,14]}

            self.relu46_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_30_, output shape: {[1024,14,14]}

            self.batchnorm47_1_30_ = gluon.nn.BatchNorm()
            # batchnorm47_1_30_, output shape: {[1024,14,14]}

            self.conv45_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_31_, output shape: {[16,28,28]}

            self.batchnorm45_1_31_ = gluon.nn.BatchNorm()
            # batchnorm45_1_31_, output shape: {[16,28,28]}

            self.relu45_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_31_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_31_, output shape: {[16,14,14]}

            self.batchnorm46_1_31_ = gluon.nn.BatchNorm()
            # batchnorm46_1_31_, output shape: {[16,14,14]}

            self.relu46_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_31_, output shape: {[1024,14,14]}

            self.batchnorm47_1_31_ = gluon.nn.BatchNorm()
            # batchnorm47_1_31_, output shape: {[1024,14,14]}

            self.conv45_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv45_1_32_, output shape: {[16,28,28]}

            self.batchnorm45_1_32_ = gluon.nn.BatchNorm()
            # batchnorm45_1_32_, output shape: {[16,28,28]}

            self.relu45_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv46_1_32_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv46_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv46_1_32_, output shape: {[16,14,14]}

            self.batchnorm46_1_32_ = gluon.nn.BatchNorm()
            # batchnorm46_1_32_, output shape: {[16,14,14]}

            self.relu46_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv47_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv47_1_32_, output shape: {[1024,14,14]}

            self.batchnorm47_1_32_ = gluon.nn.BatchNorm()
            # batchnorm47_1_32_, output shape: {[1024,14,14]}

            self.conv44_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv44_2_, output shape: {[1024,14,14]}

            self.batchnorm44_2_ = gluon.nn.BatchNorm()
            # batchnorm44_2_, output shape: {[1024,14,14]}

            self.relu49_ = gluon.nn.Activation(activation='relu')
            self.conv51_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_1_, output shape: {[16,14,14]}

            self.batchnorm51_1_1_ = gluon.nn.BatchNorm()
            # batchnorm51_1_1_, output shape: {[16,14,14]}

            self.relu51_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_1_, output shape: {[16,14,14]}

            self.batchnorm52_1_1_ = gluon.nn.BatchNorm()
            # batchnorm52_1_1_, output shape: {[16,14,14]}

            self.relu52_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_1_, output shape: {[1024,14,14]}

            self.batchnorm53_1_1_ = gluon.nn.BatchNorm()
            # batchnorm53_1_1_, output shape: {[1024,14,14]}

            self.conv51_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_2_, output shape: {[16,14,14]}

            self.batchnorm51_1_2_ = gluon.nn.BatchNorm()
            # batchnorm51_1_2_, output shape: {[16,14,14]}

            self.relu51_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_2_, output shape: {[16,14,14]}

            self.batchnorm52_1_2_ = gluon.nn.BatchNorm()
            # batchnorm52_1_2_, output shape: {[16,14,14]}

            self.relu52_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_2_, output shape: {[1024,14,14]}

            self.batchnorm53_1_2_ = gluon.nn.BatchNorm()
            # batchnorm53_1_2_, output shape: {[1024,14,14]}

            self.conv51_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_3_, output shape: {[16,14,14]}

            self.batchnorm51_1_3_ = gluon.nn.BatchNorm()
            # batchnorm51_1_3_, output shape: {[16,14,14]}

            self.relu51_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_3_, output shape: {[16,14,14]}

            self.batchnorm52_1_3_ = gluon.nn.BatchNorm()
            # batchnorm52_1_3_, output shape: {[16,14,14]}

            self.relu52_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_3_, output shape: {[1024,14,14]}

            self.batchnorm53_1_3_ = gluon.nn.BatchNorm()
            # batchnorm53_1_3_, output shape: {[1024,14,14]}

            self.conv51_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_4_, output shape: {[16,14,14]}

            self.batchnorm51_1_4_ = gluon.nn.BatchNorm()
            # batchnorm51_1_4_, output shape: {[16,14,14]}

            self.relu51_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_4_, output shape: {[16,14,14]}

            self.batchnorm52_1_4_ = gluon.nn.BatchNorm()
            # batchnorm52_1_4_, output shape: {[16,14,14]}

            self.relu52_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_4_, output shape: {[1024,14,14]}

            self.batchnorm53_1_4_ = gluon.nn.BatchNorm()
            # batchnorm53_1_4_, output shape: {[1024,14,14]}

            self.conv51_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_5_, output shape: {[16,14,14]}

            self.batchnorm51_1_5_ = gluon.nn.BatchNorm()
            # batchnorm51_1_5_, output shape: {[16,14,14]}

            self.relu51_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_5_, output shape: {[16,14,14]}

            self.batchnorm52_1_5_ = gluon.nn.BatchNorm()
            # batchnorm52_1_5_, output shape: {[16,14,14]}

            self.relu52_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_5_, output shape: {[1024,14,14]}

            self.batchnorm53_1_5_ = gluon.nn.BatchNorm()
            # batchnorm53_1_5_, output shape: {[1024,14,14]}

            self.conv51_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_6_, output shape: {[16,14,14]}

            self.batchnorm51_1_6_ = gluon.nn.BatchNorm()
            # batchnorm51_1_6_, output shape: {[16,14,14]}

            self.relu51_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_6_, output shape: {[16,14,14]}

            self.batchnorm52_1_6_ = gluon.nn.BatchNorm()
            # batchnorm52_1_6_, output shape: {[16,14,14]}

            self.relu52_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_6_, output shape: {[1024,14,14]}

            self.batchnorm53_1_6_ = gluon.nn.BatchNorm()
            # batchnorm53_1_6_, output shape: {[1024,14,14]}

            self.conv51_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_7_, output shape: {[16,14,14]}

            self.batchnorm51_1_7_ = gluon.nn.BatchNorm()
            # batchnorm51_1_7_, output shape: {[16,14,14]}

            self.relu51_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_7_, output shape: {[16,14,14]}

            self.batchnorm52_1_7_ = gluon.nn.BatchNorm()
            # batchnorm52_1_7_, output shape: {[16,14,14]}

            self.relu52_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_7_, output shape: {[1024,14,14]}

            self.batchnorm53_1_7_ = gluon.nn.BatchNorm()
            # batchnorm53_1_7_, output shape: {[1024,14,14]}

            self.conv51_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_8_, output shape: {[16,14,14]}

            self.batchnorm51_1_8_ = gluon.nn.BatchNorm()
            # batchnorm51_1_8_, output shape: {[16,14,14]}

            self.relu51_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_8_, output shape: {[16,14,14]}

            self.batchnorm52_1_8_ = gluon.nn.BatchNorm()
            # batchnorm52_1_8_, output shape: {[16,14,14]}

            self.relu52_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_8_, output shape: {[1024,14,14]}

            self.batchnorm53_1_8_ = gluon.nn.BatchNorm()
            # batchnorm53_1_8_, output shape: {[1024,14,14]}

            self.conv51_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_9_, output shape: {[16,14,14]}

            self.batchnorm51_1_9_ = gluon.nn.BatchNorm()
            # batchnorm51_1_9_, output shape: {[16,14,14]}

            self.relu51_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_9_, output shape: {[16,14,14]}

            self.batchnorm52_1_9_ = gluon.nn.BatchNorm()
            # batchnorm52_1_9_, output shape: {[16,14,14]}

            self.relu52_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_9_, output shape: {[1024,14,14]}

            self.batchnorm53_1_9_ = gluon.nn.BatchNorm()
            # batchnorm53_1_9_, output shape: {[1024,14,14]}

            self.conv51_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_10_, output shape: {[16,14,14]}

            self.batchnorm51_1_10_ = gluon.nn.BatchNorm()
            # batchnorm51_1_10_, output shape: {[16,14,14]}

            self.relu51_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_10_, output shape: {[16,14,14]}

            self.batchnorm52_1_10_ = gluon.nn.BatchNorm()
            # batchnorm52_1_10_, output shape: {[16,14,14]}

            self.relu52_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_10_, output shape: {[1024,14,14]}

            self.batchnorm53_1_10_ = gluon.nn.BatchNorm()
            # batchnorm53_1_10_, output shape: {[1024,14,14]}

            self.conv51_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_11_, output shape: {[16,14,14]}

            self.batchnorm51_1_11_ = gluon.nn.BatchNorm()
            # batchnorm51_1_11_, output shape: {[16,14,14]}

            self.relu51_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_11_, output shape: {[16,14,14]}

            self.batchnorm52_1_11_ = gluon.nn.BatchNorm()
            # batchnorm52_1_11_, output shape: {[16,14,14]}

            self.relu52_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_11_, output shape: {[1024,14,14]}

            self.batchnorm53_1_11_ = gluon.nn.BatchNorm()
            # batchnorm53_1_11_, output shape: {[1024,14,14]}

            self.conv51_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_12_, output shape: {[16,14,14]}

            self.batchnorm51_1_12_ = gluon.nn.BatchNorm()
            # batchnorm51_1_12_, output shape: {[16,14,14]}

            self.relu51_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_12_, output shape: {[16,14,14]}

            self.batchnorm52_1_12_ = gluon.nn.BatchNorm()
            # batchnorm52_1_12_, output shape: {[16,14,14]}

            self.relu52_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_12_, output shape: {[1024,14,14]}

            self.batchnorm53_1_12_ = gluon.nn.BatchNorm()
            # batchnorm53_1_12_, output shape: {[1024,14,14]}

            self.conv51_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_13_, output shape: {[16,14,14]}

            self.batchnorm51_1_13_ = gluon.nn.BatchNorm()
            # batchnorm51_1_13_, output shape: {[16,14,14]}

            self.relu51_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_13_, output shape: {[16,14,14]}

            self.batchnorm52_1_13_ = gluon.nn.BatchNorm()
            # batchnorm52_1_13_, output shape: {[16,14,14]}

            self.relu52_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_13_, output shape: {[1024,14,14]}

            self.batchnorm53_1_13_ = gluon.nn.BatchNorm()
            # batchnorm53_1_13_, output shape: {[1024,14,14]}

            self.conv51_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_14_, output shape: {[16,14,14]}

            self.batchnorm51_1_14_ = gluon.nn.BatchNorm()
            # batchnorm51_1_14_, output shape: {[16,14,14]}

            self.relu51_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_14_, output shape: {[16,14,14]}

            self.batchnorm52_1_14_ = gluon.nn.BatchNorm()
            # batchnorm52_1_14_, output shape: {[16,14,14]}

            self.relu52_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_14_, output shape: {[1024,14,14]}

            self.batchnorm53_1_14_ = gluon.nn.BatchNorm()
            # batchnorm53_1_14_, output shape: {[1024,14,14]}

            self.conv51_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_15_, output shape: {[16,14,14]}

            self.batchnorm51_1_15_ = gluon.nn.BatchNorm()
            # batchnorm51_1_15_, output shape: {[16,14,14]}

            self.relu51_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_15_, output shape: {[16,14,14]}

            self.batchnorm52_1_15_ = gluon.nn.BatchNorm()
            # batchnorm52_1_15_, output shape: {[16,14,14]}

            self.relu52_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_15_, output shape: {[1024,14,14]}

            self.batchnorm53_1_15_ = gluon.nn.BatchNorm()
            # batchnorm53_1_15_, output shape: {[1024,14,14]}

            self.conv51_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_16_, output shape: {[16,14,14]}

            self.batchnorm51_1_16_ = gluon.nn.BatchNorm()
            # batchnorm51_1_16_, output shape: {[16,14,14]}

            self.relu51_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_16_, output shape: {[16,14,14]}

            self.batchnorm52_1_16_ = gluon.nn.BatchNorm()
            # batchnorm52_1_16_, output shape: {[16,14,14]}

            self.relu52_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_16_, output shape: {[1024,14,14]}

            self.batchnorm53_1_16_ = gluon.nn.BatchNorm()
            # batchnorm53_1_16_, output shape: {[1024,14,14]}

            self.conv51_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_17_, output shape: {[16,14,14]}

            self.batchnorm51_1_17_ = gluon.nn.BatchNorm()
            # batchnorm51_1_17_, output shape: {[16,14,14]}

            self.relu51_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_17_, output shape: {[16,14,14]}

            self.batchnorm52_1_17_ = gluon.nn.BatchNorm()
            # batchnorm52_1_17_, output shape: {[16,14,14]}

            self.relu52_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_17_, output shape: {[1024,14,14]}

            self.batchnorm53_1_17_ = gluon.nn.BatchNorm()
            # batchnorm53_1_17_, output shape: {[1024,14,14]}

            self.conv51_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_18_, output shape: {[16,14,14]}

            self.batchnorm51_1_18_ = gluon.nn.BatchNorm()
            # batchnorm51_1_18_, output shape: {[16,14,14]}

            self.relu51_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_18_, output shape: {[16,14,14]}

            self.batchnorm52_1_18_ = gluon.nn.BatchNorm()
            # batchnorm52_1_18_, output shape: {[16,14,14]}

            self.relu52_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_18_, output shape: {[1024,14,14]}

            self.batchnorm53_1_18_ = gluon.nn.BatchNorm()
            # batchnorm53_1_18_, output shape: {[1024,14,14]}

            self.conv51_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_19_, output shape: {[16,14,14]}

            self.batchnorm51_1_19_ = gluon.nn.BatchNorm()
            # batchnorm51_1_19_, output shape: {[16,14,14]}

            self.relu51_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_19_, output shape: {[16,14,14]}

            self.batchnorm52_1_19_ = gluon.nn.BatchNorm()
            # batchnorm52_1_19_, output shape: {[16,14,14]}

            self.relu52_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_19_, output shape: {[1024,14,14]}

            self.batchnorm53_1_19_ = gluon.nn.BatchNorm()
            # batchnorm53_1_19_, output shape: {[1024,14,14]}

            self.conv51_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_20_, output shape: {[16,14,14]}

            self.batchnorm51_1_20_ = gluon.nn.BatchNorm()
            # batchnorm51_1_20_, output shape: {[16,14,14]}

            self.relu51_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_20_, output shape: {[16,14,14]}

            self.batchnorm52_1_20_ = gluon.nn.BatchNorm()
            # batchnorm52_1_20_, output shape: {[16,14,14]}

            self.relu52_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_20_, output shape: {[1024,14,14]}

            self.batchnorm53_1_20_ = gluon.nn.BatchNorm()
            # batchnorm53_1_20_, output shape: {[1024,14,14]}

            self.conv51_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_21_, output shape: {[16,14,14]}

            self.batchnorm51_1_21_ = gluon.nn.BatchNorm()
            # batchnorm51_1_21_, output shape: {[16,14,14]}

            self.relu51_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_21_, output shape: {[16,14,14]}

            self.batchnorm52_1_21_ = gluon.nn.BatchNorm()
            # batchnorm52_1_21_, output shape: {[16,14,14]}

            self.relu52_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_21_, output shape: {[1024,14,14]}

            self.batchnorm53_1_21_ = gluon.nn.BatchNorm()
            # batchnorm53_1_21_, output shape: {[1024,14,14]}

            self.conv51_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_22_, output shape: {[16,14,14]}

            self.batchnorm51_1_22_ = gluon.nn.BatchNorm()
            # batchnorm51_1_22_, output shape: {[16,14,14]}

            self.relu51_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_22_, output shape: {[16,14,14]}

            self.batchnorm52_1_22_ = gluon.nn.BatchNorm()
            # batchnorm52_1_22_, output shape: {[16,14,14]}

            self.relu52_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_22_, output shape: {[1024,14,14]}

            self.batchnorm53_1_22_ = gluon.nn.BatchNorm()
            # batchnorm53_1_22_, output shape: {[1024,14,14]}

            self.conv51_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_23_, output shape: {[16,14,14]}

            self.batchnorm51_1_23_ = gluon.nn.BatchNorm()
            # batchnorm51_1_23_, output shape: {[16,14,14]}

            self.relu51_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_23_, output shape: {[16,14,14]}

            self.batchnorm52_1_23_ = gluon.nn.BatchNorm()
            # batchnorm52_1_23_, output shape: {[16,14,14]}

            self.relu52_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_23_, output shape: {[1024,14,14]}

            self.batchnorm53_1_23_ = gluon.nn.BatchNorm()
            # batchnorm53_1_23_, output shape: {[1024,14,14]}

            self.conv51_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_24_, output shape: {[16,14,14]}

            self.batchnorm51_1_24_ = gluon.nn.BatchNorm()
            # batchnorm51_1_24_, output shape: {[16,14,14]}

            self.relu51_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_24_, output shape: {[16,14,14]}

            self.batchnorm52_1_24_ = gluon.nn.BatchNorm()
            # batchnorm52_1_24_, output shape: {[16,14,14]}

            self.relu52_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_24_, output shape: {[1024,14,14]}

            self.batchnorm53_1_24_ = gluon.nn.BatchNorm()
            # batchnorm53_1_24_, output shape: {[1024,14,14]}

            self.conv51_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_25_, output shape: {[16,14,14]}

            self.batchnorm51_1_25_ = gluon.nn.BatchNorm()
            # batchnorm51_1_25_, output shape: {[16,14,14]}

            self.relu51_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_25_, output shape: {[16,14,14]}

            self.batchnorm52_1_25_ = gluon.nn.BatchNorm()
            # batchnorm52_1_25_, output shape: {[16,14,14]}

            self.relu52_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_25_, output shape: {[1024,14,14]}

            self.batchnorm53_1_25_ = gluon.nn.BatchNorm()
            # batchnorm53_1_25_, output shape: {[1024,14,14]}

            self.conv51_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_26_, output shape: {[16,14,14]}

            self.batchnorm51_1_26_ = gluon.nn.BatchNorm()
            # batchnorm51_1_26_, output shape: {[16,14,14]}

            self.relu51_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_26_, output shape: {[16,14,14]}

            self.batchnorm52_1_26_ = gluon.nn.BatchNorm()
            # batchnorm52_1_26_, output shape: {[16,14,14]}

            self.relu52_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_26_, output shape: {[1024,14,14]}

            self.batchnorm53_1_26_ = gluon.nn.BatchNorm()
            # batchnorm53_1_26_, output shape: {[1024,14,14]}

            self.conv51_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_27_, output shape: {[16,14,14]}

            self.batchnorm51_1_27_ = gluon.nn.BatchNorm()
            # batchnorm51_1_27_, output shape: {[16,14,14]}

            self.relu51_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_27_, output shape: {[16,14,14]}

            self.batchnorm52_1_27_ = gluon.nn.BatchNorm()
            # batchnorm52_1_27_, output shape: {[16,14,14]}

            self.relu52_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_27_, output shape: {[1024,14,14]}

            self.batchnorm53_1_27_ = gluon.nn.BatchNorm()
            # batchnorm53_1_27_, output shape: {[1024,14,14]}

            self.conv51_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_28_, output shape: {[16,14,14]}

            self.batchnorm51_1_28_ = gluon.nn.BatchNorm()
            # batchnorm51_1_28_, output shape: {[16,14,14]}

            self.relu51_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_28_, output shape: {[16,14,14]}

            self.batchnorm52_1_28_ = gluon.nn.BatchNorm()
            # batchnorm52_1_28_, output shape: {[16,14,14]}

            self.relu52_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_28_, output shape: {[1024,14,14]}

            self.batchnorm53_1_28_ = gluon.nn.BatchNorm()
            # batchnorm53_1_28_, output shape: {[1024,14,14]}

            self.conv51_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_29_, output shape: {[16,14,14]}

            self.batchnorm51_1_29_ = gluon.nn.BatchNorm()
            # batchnorm51_1_29_, output shape: {[16,14,14]}

            self.relu51_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_29_, output shape: {[16,14,14]}

            self.batchnorm52_1_29_ = gluon.nn.BatchNorm()
            # batchnorm52_1_29_, output shape: {[16,14,14]}

            self.relu52_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_29_, output shape: {[1024,14,14]}

            self.batchnorm53_1_29_ = gluon.nn.BatchNorm()
            # batchnorm53_1_29_, output shape: {[1024,14,14]}

            self.conv51_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_30_, output shape: {[16,14,14]}

            self.batchnorm51_1_30_ = gluon.nn.BatchNorm()
            # batchnorm51_1_30_, output shape: {[16,14,14]}

            self.relu51_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_30_, output shape: {[16,14,14]}

            self.batchnorm52_1_30_ = gluon.nn.BatchNorm()
            # batchnorm52_1_30_, output shape: {[16,14,14]}

            self.relu52_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_30_, output shape: {[1024,14,14]}

            self.batchnorm53_1_30_ = gluon.nn.BatchNorm()
            # batchnorm53_1_30_, output shape: {[1024,14,14]}

            self.conv51_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_31_, output shape: {[16,14,14]}

            self.batchnorm51_1_31_ = gluon.nn.BatchNorm()
            # batchnorm51_1_31_, output shape: {[16,14,14]}

            self.relu51_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_31_, output shape: {[16,14,14]}

            self.batchnorm52_1_31_ = gluon.nn.BatchNorm()
            # batchnorm52_1_31_, output shape: {[16,14,14]}

            self.relu52_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_31_, output shape: {[1024,14,14]}

            self.batchnorm53_1_31_ = gluon.nn.BatchNorm()
            # batchnorm53_1_31_, output shape: {[1024,14,14]}

            self.conv51_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv51_1_32_, output shape: {[16,14,14]}

            self.batchnorm51_1_32_ = gluon.nn.BatchNorm()
            # batchnorm51_1_32_, output shape: {[16,14,14]}

            self.relu51_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv52_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv52_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv52_1_32_, output shape: {[16,14,14]}

            self.batchnorm52_1_32_ = gluon.nn.BatchNorm()
            # batchnorm52_1_32_, output shape: {[16,14,14]}

            self.relu52_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv53_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv53_1_32_, output shape: {[1024,14,14]}

            self.batchnorm53_1_32_ = gluon.nn.BatchNorm()
            # batchnorm53_1_32_, output shape: {[1024,14,14]}

            self.relu55_ = gluon.nn.Activation(activation='relu')
            self.conv57_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_1_, output shape: {[16,14,14]}

            self.batchnorm57_1_1_ = gluon.nn.BatchNorm()
            # batchnorm57_1_1_, output shape: {[16,14,14]}

            self.relu57_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_1_, output shape: {[16,14,14]}

            self.batchnorm58_1_1_ = gluon.nn.BatchNorm()
            # batchnorm58_1_1_, output shape: {[16,14,14]}

            self.relu58_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_1_, output shape: {[1024,14,14]}

            self.batchnorm59_1_1_ = gluon.nn.BatchNorm()
            # batchnorm59_1_1_, output shape: {[1024,14,14]}

            self.conv57_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_2_, output shape: {[16,14,14]}

            self.batchnorm57_1_2_ = gluon.nn.BatchNorm()
            # batchnorm57_1_2_, output shape: {[16,14,14]}

            self.relu57_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_2_, output shape: {[16,14,14]}

            self.batchnorm58_1_2_ = gluon.nn.BatchNorm()
            # batchnorm58_1_2_, output shape: {[16,14,14]}

            self.relu58_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_2_, output shape: {[1024,14,14]}

            self.batchnorm59_1_2_ = gluon.nn.BatchNorm()
            # batchnorm59_1_2_, output shape: {[1024,14,14]}

            self.conv57_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_3_, output shape: {[16,14,14]}

            self.batchnorm57_1_3_ = gluon.nn.BatchNorm()
            # batchnorm57_1_3_, output shape: {[16,14,14]}

            self.relu57_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_3_, output shape: {[16,14,14]}

            self.batchnorm58_1_3_ = gluon.nn.BatchNorm()
            # batchnorm58_1_3_, output shape: {[16,14,14]}

            self.relu58_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_3_, output shape: {[1024,14,14]}

            self.batchnorm59_1_3_ = gluon.nn.BatchNorm()
            # batchnorm59_1_3_, output shape: {[1024,14,14]}

            self.conv57_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_4_, output shape: {[16,14,14]}

            self.batchnorm57_1_4_ = gluon.nn.BatchNorm()
            # batchnorm57_1_4_, output shape: {[16,14,14]}

            self.relu57_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_4_, output shape: {[16,14,14]}

            self.batchnorm58_1_4_ = gluon.nn.BatchNorm()
            # batchnorm58_1_4_, output shape: {[16,14,14]}

            self.relu58_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_4_, output shape: {[1024,14,14]}

            self.batchnorm59_1_4_ = gluon.nn.BatchNorm()
            # batchnorm59_1_4_, output shape: {[1024,14,14]}

            self.conv57_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_5_, output shape: {[16,14,14]}

            self.batchnorm57_1_5_ = gluon.nn.BatchNorm()
            # batchnorm57_1_5_, output shape: {[16,14,14]}

            self.relu57_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_5_, output shape: {[16,14,14]}

            self.batchnorm58_1_5_ = gluon.nn.BatchNorm()
            # batchnorm58_1_5_, output shape: {[16,14,14]}

            self.relu58_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_5_, output shape: {[1024,14,14]}

            self.batchnorm59_1_5_ = gluon.nn.BatchNorm()
            # batchnorm59_1_5_, output shape: {[1024,14,14]}

            self.conv57_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_6_, output shape: {[16,14,14]}

            self.batchnorm57_1_6_ = gluon.nn.BatchNorm()
            # batchnorm57_1_6_, output shape: {[16,14,14]}

            self.relu57_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_6_, output shape: {[16,14,14]}

            self.batchnorm58_1_6_ = gluon.nn.BatchNorm()
            # batchnorm58_1_6_, output shape: {[16,14,14]}

            self.relu58_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_6_, output shape: {[1024,14,14]}

            self.batchnorm59_1_6_ = gluon.nn.BatchNorm()
            # batchnorm59_1_6_, output shape: {[1024,14,14]}

            self.conv57_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_7_, output shape: {[16,14,14]}

            self.batchnorm57_1_7_ = gluon.nn.BatchNorm()
            # batchnorm57_1_7_, output shape: {[16,14,14]}

            self.relu57_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_7_, output shape: {[16,14,14]}

            self.batchnorm58_1_7_ = gluon.nn.BatchNorm()
            # batchnorm58_1_7_, output shape: {[16,14,14]}

            self.relu58_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_7_, output shape: {[1024,14,14]}

            self.batchnorm59_1_7_ = gluon.nn.BatchNorm()
            # batchnorm59_1_7_, output shape: {[1024,14,14]}

            self.conv57_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_8_, output shape: {[16,14,14]}

            self.batchnorm57_1_8_ = gluon.nn.BatchNorm()
            # batchnorm57_1_8_, output shape: {[16,14,14]}

            self.relu57_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_8_, output shape: {[16,14,14]}

            self.batchnorm58_1_8_ = gluon.nn.BatchNorm()
            # batchnorm58_1_8_, output shape: {[16,14,14]}

            self.relu58_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_8_, output shape: {[1024,14,14]}

            self.batchnorm59_1_8_ = gluon.nn.BatchNorm()
            # batchnorm59_1_8_, output shape: {[1024,14,14]}

            self.conv57_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_9_, output shape: {[16,14,14]}

            self.batchnorm57_1_9_ = gluon.nn.BatchNorm()
            # batchnorm57_1_9_, output shape: {[16,14,14]}

            self.relu57_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_9_, output shape: {[16,14,14]}

            self.batchnorm58_1_9_ = gluon.nn.BatchNorm()
            # batchnorm58_1_9_, output shape: {[16,14,14]}

            self.relu58_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_9_, output shape: {[1024,14,14]}

            self.batchnorm59_1_9_ = gluon.nn.BatchNorm()
            # batchnorm59_1_9_, output shape: {[1024,14,14]}

            self.conv57_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_10_, output shape: {[16,14,14]}

            self.batchnorm57_1_10_ = gluon.nn.BatchNorm()
            # batchnorm57_1_10_, output shape: {[16,14,14]}

            self.relu57_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_10_, output shape: {[16,14,14]}

            self.batchnorm58_1_10_ = gluon.nn.BatchNorm()
            # batchnorm58_1_10_, output shape: {[16,14,14]}

            self.relu58_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_10_, output shape: {[1024,14,14]}

            self.batchnorm59_1_10_ = gluon.nn.BatchNorm()
            # batchnorm59_1_10_, output shape: {[1024,14,14]}

            self.conv57_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_11_, output shape: {[16,14,14]}

            self.batchnorm57_1_11_ = gluon.nn.BatchNorm()
            # batchnorm57_1_11_, output shape: {[16,14,14]}

            self.relu57_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_11_, output shape: {[16,14,14]}

            self.batchnorm58_1_11_ = gluon.nn.BatchNorm()
            # batchnorm58_1_11_, output shape: {[16,14,14]}

            self.relu58_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_11_, output shape: {[1024,14,14]}

            self.batchnorm59_1_11_ = gluon.nn.BatchNorm()
            # batchnorm59_1_11_, output shape: {[1024,14,14]}

            self.conv57_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_12_, output shape: {[16,14,14]}

            self.batchnorm57_1_12_ = gluon.nn.BatchNorm()
            # batchnorm57_1_12_, output shape: {[16,14,14]}

            self.relu57_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_12_, output shape: {[16,14,14]}

            self.batchnorm58_1_12_ = gluon.nn.BatchNorm()
            # batchnorm58_1_12_, output shape: {[16,14,14]}

            self.relu58_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_12_, output shape: {[1024,14,14]}

            self.batchnorm59_1_12_ = gluon.nn.BatchNorm()
            # batchnorm59_1_12_, output shape: {[1024,14,14]}

            self.conv57_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_13_, output shape: {[16,14,14]}

            self.batchnorm57_1_13_ = gluon.nn.BatchNorm()
            # batchnorm57_1_13_, output shape: {[16,14,14]}

            self.relu57_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_13_, output shape: {[16,14,14]}

            self.batchnorm58_1_13_ = gluon.nn.BatchNorm()
            # batchnorm58_1_13_, output shape: {[16,14,14]}

            self.relu58_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_13_, output shape: {[1024,14,14]}

            self.batchnorm59_1_13_ = gluon.nn.BatchNorm()
            # batchnorm59_1_13_, output shape: {[1024,14,14]}

            self.conv57_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_14_, output shape: {[16,14,14]}

            self.batchnorm57_1_14_ = gluon.nn.BatchNorm()
            # batchnorm57_1_14_, output shape: {[16,14,14]}

            self.relu57_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_14_, output shape: {[16,14,14]}

            self.batchnorm58_1_14_ = gluon.nn.BatchNorm()
            # batchnorm58_1_14_, output shape: {[16,14,14]}

            self.relu58_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_14_, output shape: {[1024,14,14]}

            self.batchnorm59_1_14_ = gluon.nn.BatchNorm()
            # batchnorm59_1_14_, output shape: {[1024,14,14]}

            self.conv57_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_15_, output shape: {[16,14,14]}

            self.batchnorm57_1_15_ = gluon.nn.BatchNorm()
            # batchnorm57_1_15_, output shape: {[16,14,14]}

            self.relu57_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_15_, output shape: {[16,14,14]}

            self.batchnorm58_1_15_ = gluon.nn.BatchNorm()
            # batchnorm58_1_15_, output shape: {[16,14,14]}

            self.relu58_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_15_, output shape: {[1024,14,14]}

            self.batchnorm59_1_15_ = gluon.nn.BatchNorm()
            # batchnorm59_1_15_, output shape: {[1024,14,14]}

            self.conv57_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_16_, output shape: {[16,14,14]}

            self.batchnorm57_1_16_ = gluon.nn.BatchNorm()
            # batchnorm57_1_16_, output shape: {[16,14,14]}

            self.relu57_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_16_, output shape: {[16,14,14]}

            self.batchnorm58_1_16_ = gluon.nn.BatchNorm()
            # batchnorm58_1_16_, output shape: {[16,14,14]}

            self.relu58_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_16_, output shape: {[1024,14,14]}

            self.batchnorm59_1_16_ = gluon.nn.BatchNorm()
            # batchnorm59_1_16_, output shape: {[1024,14,14]}

            self.conv57_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_17_, output shape: {[16,14,14]}

            self.batchnorm57_1_17_ = gluon.nn.BatchNorm()
            # batchnorm57_1_17_, output shape: {[16,14,14]}

            self.relu57_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_17_, output shape: {[16,14,14]}

            self.batchnorm58_1_17_ = gluon.nn.BatchNorm()
            # batchnorm58_1_17_, output shape: {[16,14,14]}

            self.relu58_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_17_, output shape: {[1024,14,14]}

            self.batchnorm59_1_17_ = gluon.nn.BatchNorm()
            # batchnorm59_1_17_, output shape: {[1024,14,14]}

            self.conv57_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_18_, output shape: {[16,14,14]}

            self.batchnorm57_1_18_ = gluon.nn.BatchNorm()
            # batchnorm57_1_18_, output shape: {[16,14,14]}

            self.relu57_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_18_, output shape: {[16,14,14]}

            self.batchnorm58_1_18_ = gluon.nn.BatchNorm()
            # batchnorm58_1_18_, output shape: {[16,14,14]}

            self.relu58_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_18_, output shape: {[1024,14,14]}

            self.batchnorm59_1_18_ = gluon.nn.BatchNorm()
            # batchnorm59_1_18_, output shape: {[1024,14,14]}

            self.conv57_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_19_, output shape: {[16,14,14]}

            self.batchnorm57_1_19_ = gluon.nn.BatchNorm()
            # batchnorm57_1_19_, output shape: {[16,14,14]}

            self.relu57_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_19_, output shape: {[16,14,14]}

            self.batchnorm58_1_19_ = gluon.nn.BatchNorm()
            # batchnorm58_1_19_, output shape: {[16,14,14]}

            self.relu58_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_19_, output shape: {[1024,14,14]}

            self.batchnorm59_1_19_ = gluon.nn.BatchNorm()
            # batchnorm59_1_19_, output shape: {[1024,14,14]}

            self.conv57_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_20_, output shape: {[16,14,14]}

            self.batchnorm57_1_20_ = gluon.nn.BatchNorm()
            # batchnorm57_1_20_, output shape: {[16,14,14]}

            self.relu57_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_20_, output shape: {[16,14,14]}

            self.batchnorm58_1_20_ = gluon.nn.BatchNorm()
            # batchnorm58_1_20_, output shape: {[16,14,14]}

            self.relu58_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_20_, output shape: {[1024,14,14]}

            self.batchnorm59_1_20_ = gluon.nn.BatchNorm()
            # batchnorm59_1_20_, output shape: {[1024,14,14]}

            self.conv57_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_21_, output shape: {[16,14,14]}

            self.batchnorm57_1_21_ = gluon.nn.BatchNorm()
            # batchnorm57_1_21_, output shape: {[16,14,14]}

            self.relu57_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_21_, output shape: {[16,14,14]}

            self.batchnorm58_1_21_ = gluon.nn.BatchNorm()
            # batchnorm58_1_21_, output shape: {[16,14,14]}

            self.relu58_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_21_, output shape: {[1024,14,14]}

            self.batchnorm59_1_21_ = gluon.nn.BatchNorm()
            # batchnorm59_1_21_, output shape: {[1024,14,14]}

            self.conv57_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_22_, output shape: {[16,14,14]}

            self.batchnorm57_1_22_ = gluon.nn.BatchNorm()
            # batchnorm57_1_22_, output shape: {[16,14,14]}

            self.relu57_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_22_, output shape: {[16,14,14]}

            self.batchnorm58_1_22_ = gluon.nn.BatchNorm()
            # batchnorm58_1_22_, output shape: {[16,14,14]}

            self.relu58_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_22_, output shape: {[1024,14,14]}

            self.batchnorm59_1_22_ = gluon.nn.BatchNorm()
            # batchnorm59_1_22_, output shape: {[1024,14,14]}

            self.conv57_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_23_, output shape: {[16,14,14]}

            self.batchnorm57_1_23_ = gluon.nn.BatchNorm()
            # batchnorm57_1_23_, output shape: {[16,14,14]}

            self.relu57_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_23_, output shape: {[16,14,14]}

            self.batchnorm58_1_23_ = gluon.nn.BatchNorm()
            # batchnorm58_1_23_, output shape: {[16,14,14]}

            self.relu58_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_23_, output shape: {[1024,14,14]}

            self.batchnorm59_1_23_ = gluon.nn.BatchNorm()
            # batchnorm59_1_23_, output shape: {[1024,14,14]}

            self.conv57_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_24_, output shape: {[16,14,14]}

            self.batchnorm57_1_24_ = gluon.nn.BatchNorm()
            # batchnorm57_1_24_, output shape: {[16,14,14]}

            self.relu57_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_24_, output shape: {[16,14,14]}

            self.batchnorm58_1_24_ = gluon.nn.BatchNorm()
            # batchnorm58_1_24_, output shape: {[16,14,14]}

            self.relu58_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_24_, output shape: {[1024,14,14]}

            self.batchnorm59_1_24_ = gluon.nn.BatchNorm()
            # batchnorm59_1_24_, output shape: {[1024,14,14]}

            self.conv57_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_25_, output shape: {[16,14,14]}

            self.batchnorm57_1_25_ = gluon.nn.BatchNorm()
            # batchnorm57_1_25_, output shape: {[16,14,14]}

            self.relu57_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_25_, output shape: {[16,14,14]}

            self.batchnorm58_1_25_ = gluon.nn.BatchNorm()
            # batchnorm58_1_25_, output shape: {[16,14,14]}

            self.relu58_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_25_, output shape: {[1024,14,14]}

            self.batchnorm59_1_25_ = gluon.nn.BatchNorm()
            # batchnorm59_1_25_, output shape: {[1024,14,14]}

            self.conv57_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_26_, output shape: {[16,14,14]}

            self.batchnorm57_1_26_ = gluon.nn.BatchNorm()
            # batchnorm57_1_26_, output shape: {[16,14,14]}

            self.relu57_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_26_, output shape: {[16,14,14]}

            self.batchnorm58_1_26_ = gluon.nn.BatchNorm()
            # batchnorm58_1_26_, output shape: {[16,14,14]}

            self.relu58_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_26_, output shape: {[1024,14,14]}

            self.batchnorm59_1_26_ = gluon.nn.BatchNorm()
            # batchnorm59_1_26_, output shape: {[1024,14,14]}

            self.conv57_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_27_, output shape: {[16,14,14]}

            self.batchnorm57_1_27_ = gluon.nn.BatchNorm()
            # batchnorm57_1_27_, output shape: {[16,14,14]}

            self.relu57_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_27_, output shape: {[16,14,14]}

            self.batchnorm58_1_27_ = gluon.nn.BatchNorm()
            # batchnorm58_1_27_, output shape: {[16,14,14]}

            self.relu58_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_27_, output shape: {[1024,14,14]}

            self.batchnorm59_1_27_ = gluon.nn.BatchNorm()
            # batchnorm59_1_27_, output shape: {[1024,14,14]}

            self.conv57_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_28_, output shape: {[16,14,14]}

            self.batchnorm57_1_28_ = gluon.nn.BatchNorm()
            # batchnorm57_1_28_, output shape: {[16,14,14]}

            self.relu57_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_28_, output shape: {[16,14,14]}

            self.batchnorm58_1_28_ = gluon.nn.BatchNorm()
            # batchnorm58_1_28_, output shape: {[16,14,14]}

            self.relu58_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_28_, output shape: {[1024,14,14]}

            self.batchnorm59_1_28_ = gluon.nn.BatchNorm()
            # batchnorm59_1_28_, output shape: {[1024,14,14]}

            self.conv57_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_29_, output shape: {[16,14,14]}

            self.batchnorm57_1_29_ = gluon.nn.BatchNorm()
            # batchnorm57_1_29_, output shape: {[16,14,14]}

            self.relu57_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_29_, output shape: {[16,14,14]}

            self.batchnorm58_1_29_ = gluon.nn.BatchNorm()
            # batchnorm58_1_29_, output shape: {[16,14,14]}

            self.relu58_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_29_, output shape: {[1024,14,14]}

            self.batchnorm59_1_29_ = gluon.nn.BatchNorm()
            # batchnorm59_1_29_, output shape: {[1024,14,14]}

            self.conv57_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_30_, output shape: {[16,14,14]}

            self.batchnorm57_1_30_ = gluon.nn.BatchNorm()
            # batchnorm57_1_30_, output shape: {[16,14,14]}

            self.relu57_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_30_, output shape: {[16,14,14]}

            self.batchnorm58_1_30_ = gluon.nn.BatchNorm()
            # batchnorm58_1_30_, output shape: {[16,14,14]}

            self.relu58_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_30_, output shape: {[1024,14,14]}

            self.batchnorm59_1_30_ = gluon.nn.BatchNorm()
            # batchnorm59_1_30_, output shape: {[1024,14,14]}

            self.conv57_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_31_, output shape: {[16,14,14]}

            self.batchnorm57_1_31_ = gluon.nn.BatchNorm()
            # batchnorm57_1_31_, output shape: {[16,14,14]}

            self.relu57_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_31_, output shape: {[16,14,14]}

            self.batchnorm58_1_31_ = gluon.nn.BatchNorm()
            # batchnorm58_1_31_, output shape: {[16,14,14]}

            self.relu58_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_31_, output shape: {[1024,14,14]}

            self.batchnorm59_1_31_ = gluon.nn.BatchNorm()
            # batchnorm59_1_31_, output shape: {[1024,14,14]}

            self.conv57_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv57_1_32_, output shape: {[16,14,14]}

            self.batchnorm57_1_32_ = gluon.nn.BatchNorm()
            # batchnorm57_1_32_, output shape: {[16,14,14]}

            self.relu57_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv58_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv58_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv58_1_32_, output shape: {[16,14,14]}

            self.batchnorm58_1_32_ = gluon.nn.BatchNorm()
            # batchnorm58_1_32_, output shape: {[16,14,14]}

            self.relu58_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv59_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv59_1_32_, output shape: {[1024,14,14]}

            self.batchnorm59_1_32_ = gluon.nn.BatchNorm()
            # batchnorm59_1_32_, output shape: {[1024,14,14]}

            self.relu61_ = gluon.nn.Activation(activation='relu')
            self.conv63_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_1_, output shape: {[16,14,14]}

            self.batchnorm63_1_1_ = gluon.nn.BatchNorm()
            # batchnorm63_1_1_, output shape: {[16,14,14]}

            self.relu63_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_1_, output shape: {[16,14,14]}

            self.batchnorm64_1_1_ = gluon.nn.BatchNorm()
            # batchnorm64_1_1_, output shape: {[16,14,14]}

            self.relu64_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_1_, output shape: {[1024,14,14]}

            self.batchnorm65_1_1_ = gluon.nn.BatchNorm()
            # batchnorm65_1_1_, output shape: {[1024,14,14]}

            self.conv63_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_2_, output shape: {[16,14,14]}

            self.batchnorm63_1_2_ = gluon.nn.BatchNorm()
            # batchnorm63_1_2_, output shape: {[16,14,14]}

            self.relu63_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_2_, output shape: {[16,14,14]}

            self.batchnorm64_1_2_ = gluon.nn.BatchNorm()
            # batchnorm64_1_2_, output shape: {[16,14,14]}

            self.relu64_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_2_, output shape: {[1024,14,14]}

            self.batchnorm65_1_2_ = gluon.nn.BatchNorm()
            # batchnorm65_1_2_, output shape: {[1024,14,14]}

            self.conv63_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_3_, output shape: {[16,14,14]}

            self.batchnorm63_1_3_ = gluon.nn.BatchNorm()
            # batchnorm63_1_3_, output shape: {[16,14,14]}

            self.relu63_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_3_, output shape: {[16,14,14]}

            self.batchnorm64_1_3_ = gluon.nn.BatchNorm()
            # batchnorm64_1_3_, output shape: {[16,14,14]}

            self.relu64_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_3_, output shape: {[1024,14,14]}

            self.batchnorm65_1_3_ = gluon.nn.BatchNorm()
            # batchnorm65_1_3_, output shape: {[1024,14,14]}

            self.conv63_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_4_, output shape: {[16,14,14]}

            self.batchnorm63_1_4_ = gluon.nn.BatchNorm()
            # batchnorm63_1_4_, output shape: {[16,14,14]}

            self.relu63_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_4_, output shape: {[16,14,14]}

            self.batchnorm64_1_4_ = gluon.nn.BatchNorm()
            # batchnorm64_1_4_, output shape: {[16,14,14]}

            self.relu64_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_4_, output shape: {[1024,14,14]}

            self.batchnorm65_1_4_ = gluon.nn.BatchNorm()
            # batchnorm65_1_4_, output shape: {[1024,14,14]}

            self.conv63_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_5_, output shape: {[16,14,14]}

            self.batchnorm63_1_5_ = gluon.nn.BatchNorm()
            # batchnorm63_1_5_, output shape: {[16,14,14]}

            self.relu63_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_5_, output shape: {[16,14,14]}

            self.batchnorm64_1_5_ = gluon.nn.BatchNorm()
            # batchnorm64_1_5_, output shape: {[16,14,14]}

            self.relu64_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_5_, output shape: {[1024,14,14]}

            self.batchnorm65_1_5_ = gluon.nn.BatchNorm()
            # batchnorm65_1_5_, output shape: {[1024,14,14]}

            self.conv63_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_6_, output shape: {[16,14,14]}

            self.batchnorm63_1_6_ = gluon.nn.BatchNorm()
            # batchnorm63_1_6_, output shape: {[16,14,14]}

            self.relu63_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_6_, output shape: {[16,14,14]}

            self.batchnorm64_1_6_ = gluon.nn.BatchNorm()
            # batchnorm64_1_6_, output shape: {[16,14,14]}

            self.relu64_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_6_, output shape: {[1024,14,14]}

            self.batchnorm65_1_6_ = gluon.nn.BatchNorm()
            # batchnorm65_1_6_, output shape: {[1024,14,14]}

            self.conv63_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_7_, output shape: {[16,14,14]}

            self.batchnorm63_1_7_ = gluon.nn.BatchNorm()
            # batchnorm63_1_7_, output shape: {[16,14,14]}

            self.relu63_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_7_, output shape: {[16,14,14]}

            self.batchnorm64_1_7_ = gluon.nn.BatchNorm()
            # batchnorm64_1_7_, output shape: {[16,14,14]}

            self.relu64_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_7_, output shape: {[1024,14,14]}

            self.batchnorm65_1_7_ = gluon.nn.BatchNorm()
            # batchnorm65_1_7_, output shape: {[1024,14,14]}

            self.conv63_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_8_, output shape: {[16,14,14]}

            self.batchnorm63_1_8_ = gluon.nn.BatchNorm()
            # batchnorm63_1_8_, output shape: {[16,14,14]}

            self.relu63_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_8_, output shape: {[16,14,14]}

            self.batchnorm64_1_8_ = gluon.nn.BatchNorm()
            # batchnorm64_1_8_, output shape: {[16,14,14]}

            self.relu64_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_8_, output shape: {[1024,14,14]}

            self.batchnorm65_1_8_ = gluon.nn.BatchNorm()
            # batchnorm65_1_8_, output shape: {[1024,14,14]}

            self.conv63_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_9_, output shape: {[16,14,14]}

            self.batchnorm63_1_9_ = gluon.nn.BatchNorm()
            # batchnorm63_1_9_, output shape: {[16,14,14]}

            self.relu63_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_9_, output shape: {[16,14,14]}

            self.batchnorm64_1_9_ = gluon.nn.BatchNorm()
            # batchnorm64_1_9_, output shape: {[16,14,14]}

            self.relu64_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_9_, output shape: {[1024,14,14]}

            self.batchnorm65_1_9_ = gluon.nn.BatchNorm()
            # batchnorm65_1_9_, output shape: {[1024,14,14]}

            self.conv63_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_10_, output shape: {[16,14,14]}

            self.batchnorm63_1_10_ = gluon.nn.BatchNorm()
            # batchnorm63_1_10_, output shape: {[16,14,14]}

            self.relu63_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_10_, output shape: {[16,14,14]}

            self.batchnorm64_1_10_ = gluon.nn.BatchNorm()
            # batchnorm64_1_10_, output shape: {[16,14,14]}

            self.relu64_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_10_, output shape: {[1024,14,14]}

            self.batchnorm65_1_10_ = gluon.nn.BatchNorm()
            # batchnorm65_1_10_, output shape: {[1024,14,14]}

            self.conv63_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_11_, output shape: {[16,14,14]}

            self.batchnorm63_1_11_ = gluon.nn.BatchNorm()
            # batchnorm63_1_11_, output shape: {[16,14,14]}

            self.relu63_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_11_, output shape: {[16,14,14]}

            self.batchnorm64_1_11_ = gluon.nn.BatchNorm()
            # batchnorm64_1_11_, output shape: {[16,14,14]}

            self.relu64_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_11_, output shape: {[1024,14,14]}

            self.batchnorm65_1_11_ = gluon.nn.BatchNorm()
            # batchnorm65_1_11_, output shape: {[1024,14,14]}

            self.conv63_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_12_, output shape: {[16,14,14]}

            self.batchnorm63_1_12_ = gluon.nn.BatchNorm()
            # batchnorm63_1_12_, output shape: {[16,14,14]}

            self.relu63_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_12_, output shape: {[16,14,14]}

            self.batchnorm64_1_12_ = gluon.nn.BatchNorm()
            # batchnorm64_1_12_, output shape: {[16,14,14]}

            self.relu64_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_12_, output shape: {[1024,14,14]}

            self.batchnorm65_1_12_ = gluon.nn.BatchNorm()
            # batchnorm65_1_12_, output shape: {[1024,14,14]}

            self.conv63_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_13_, output shape: {[16,14,14]}

            self.batchnorm63_1_13_ = gluon.nn.BatchNorm()
            # batchnorm63_1_13_, output shape: {[16,14,14]}

            self.relu63_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_13_, output shape: {[16,14,14]}

            self.batchnorm64_1_13_ = gluon.nn.BatchNorm()
            # batchnorm64_1_13_, output shape: {[16,14,14]}

            self.relu64_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_13_, output shape: {[1024,14,14]}

            self.batchnorm65_1_13_ = gluon.nn.BatchNorm()
            # batchnorm65_1_13_, output shape: {[1024,14,14]}

            self.conv63_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_14_, output shape: {[16,14,14]}

            self.batchnorm63_1_14_ = gluon.nn.BatchNorm()
            # batchnorm63_1_14_, output shape: {[16,14,14]}

            self.relu63_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_14_, output shape: {[16,14,14]}

            self.batchnorm64_1_14_ = gluon.nn.BatchNorm()
            # batchnorm64_1_14_, output shape: {[16,14,14]}

            self.relu64_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_14_, output shape: {[1024,14,14]}

            self.batchnorm65_1_14_ = gluon.nn.BatchNorm()
            # batchnorm65_1_14_, output shape: {[1024,14,14]}

            self.conv63_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_15_, output shape: {[16,14,14]}

            self.batchnorm63_1_15_ = gluon.nn.BatchNorm()
            # batchnorm63_1_15_, output shape: {[16,14,14]}

            self.relu63_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_15_, output shape: {[16,14,14]}

            self.batchnorm64_1_15_ = gluon.nn.BatchNorm()
            # batchnorm64_1_15_, output shape: {[16,14,14]}

            self.relu64_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_15_, output shape: {[1024,14,14]}

            self.batchnorm65_1_15_ = gluon.nn.BatchNorm()
            # batchnorm65_1_15_, output shape: {[1024,14,14]}

            self.conv63_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_16_, output shape: {[16,14,14]}

            self.batchnorm63_1_16_ = gluon.nn.BatchNorm()
            # batchnorm63_1_16_, output shape: {[16,14,14]}

            self.relu63_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_16_, output shape: {[16,14,14]}

            self.batchnorm64_1_16_ = gluon.nn.BatchNorm()
            # batchnorm64_1_16_, output shape: {[16,14,14]}

            self.relu64_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_16_, output shape: {[1024,14,14]}

            self.batchnorm65_1_16_ = gluon.nn.BatchNorm()
            # batchnorm65_1_16_, output shape: {[1024,14,14]}

            self.conv63_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_17_, output shape: {[16,14,14]}

            self.batchnorm63_1_17_ = gluon.nn.BatchNorm()
            # batchnorm63_1_17_, output shape: {[16,14,14]}

            self.relu63_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_17_, output shape: {[16,14,14]}

            self.batchnorm64_1_17_ = gluon.nn.BatchNorm()
            # batchnorm64_1_17_, output shape: {[16,14,14]}

            self.relu64_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_17_, output shape: {[1024,14,14]}

            self.batchnorm65_1_17_ = gluon.nn.BatchNorm()
            # batchnorm65_1_17_, output shape: {[1024,14,14]}

            self.conv63_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_18_, output shape: {[16,14,14]}

            self.batchnorm63_1_18_ = gluon.nn.BatchNorm()
            # batchnorm63_1_18_, output shape: {[16,14,14]}

            self.relu63_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_18_, output shape: {[16,14,14]}

            self.batchnorm64_1_18_ = gluon.nn.BatchNorm()
            # batchnorm64_1_18_, output shape: {[16,14,14]}

            self.relu64_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_18_, output shape: {[1024,14,14]}

            self.batchnorm65_1_18_ = gluon.nn.BatchNorm()
            # batchnorm65_1_18_, output shape: {[1024,14,14]}

            self.conv63_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_19_, output shape: {[16,14,14]}

            self.batchnorm63_1_19_ = gluon.nn.BatchNorm()
            # batchnorm63_1_19_, output shape: {[16,14,14]}

            self.relu63_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_19_, output shape: {[16,14,14]}

            self.batchnorm64_1_19_ = gluon.nn.BatchNorm()
            # batchnorm64_1_19_, output shape: {[16,14,14]}

            self.relu64_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_19_, output shape: {[1024,14,14]}

            self.batchnorm65_1_19_ = gluon.nn.BatchNorm()
            # batchnorm65_1_19_, output shape: {[1024,14,14]}

            self.conv63_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_20_, output shape: {[16,14,14]}

            self.batchnorm63_1_20_ = gluon.nn.BatchNorm()
            # batchnorm63_1_20_, output shape: {[16,14,14]}

            self.relu63_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_20_, output shape: {[16,14,14]}

            self.batchnorm64_1_20_ = gluon.nn.BatchNorm()
            # batchnorm64_1_20_, output shape: {[16,14,14]}

            self.relu64_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_20_, output shape: {[1024,14,14]}

            self.batchnorm65_1_20_ = gluon.nn.BatchNorm()
            # batchnorm65_1_20_, output shape: {[1024,14,14]}

            self.conv63_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_21_, output shape: {[16,14,14]}

            self.batchnorm63_1_21_ = gluon.nn.BatchNorm()
            # batchnorm63_1_21_, output shape: {[16,14,14]}

            self.relu63_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_21_, output shape: {[16,14,14]}

            self.batchnorm64_1_21_ = gluon.nn.BatchNorm()
            # batchnorm64_1_21_, output shape: {[16,14,14]}

            self.relu64_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_21_, output shape: {[1024,14,14]}

            self.batchnorm65_1_21_ = gluon.nn.BatchNorm()
            # batchnorm65_1_21_, output shape: {[1024,14,14]}

            self.conv63_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_22_, output shape: {[16,14,14]}

            self.batchnorm63_1_22_ = gluon.nn.BatchNorm()
            # batchnorm63_1_22_, output shape: {[16,14,14]}

            self.relu63_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_22_, output shape: {[16,14,14]}

            self.batchnorm64_1_22_ = gluon.nn.BatchNorm()
            # batchnorm64_1_22_, output shape: {[16,14,14]}

            self.relu64_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_22_, output shape: {[1024,14,14]}

            self.batchnorm65_1_22_ = gluon.nn.BatchNorm()
            # batchnorm65_1_22_, output shape: {[1024,14,14]}

            self.conv63_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_23_, output shape: {[16,14,14]}

            self.batchnorm63_1_23_ = gluon.nn.BatchNorm()
            # batchnorm63_1_23_, output shape: {[16,14,14]}

            self.relu63_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_23_, output shape: {[16,14,14]}

            self.batchnorm64_1_23_ = gluon.nn.BatchNorm()
            # batchnorm64_1_23_, output shape: {[16,14,14]}

            self.relu64_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_23_, output shape: {[1024,14,14]}

            self.batchnorm65_1_23_ = gluon.nn.BatchNorm()
            # batchnorm65_1_23_, output shape: {[1024,14,14]}

            self.conv63_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_24_, output shape: {[16,14,14]}

            self.batchnorm63_1_24_ = gluon.nn.BatchNorm()
            # batchnorm63_1_24_, output shape: {[16,14,14]}

            self.relu63_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_24_, output shape: {[16,14,14]}

            self.batchnorm64_1_24_ = gluon.nn.BatchNorm()
            # batchnorm64_1_24_, output shape: {[16,14,14]}

            self.relu64_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_24_, output shape: {[1024,14,14]}

            self.batchnorm65_1_24_ = gluon.nn.BatchNorm()
            # batchnorm65_1_24_, output shape: {[1024,14,14]}

            self.conv63_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_25_, output shape: {[16,14,14]}

            self.batchnorm63_1_25_ = gluon.nn.BatchNorm()
            # batchnorm63_1_25_, output shape: {[16,14,14]}

            self.relu63_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_25_, output shape: {[16,14,14]}

            self.batchnorm64_1_25_ = gluon.nn.BatchNorm()
            # batchnorm64_1_25_, output shape: {[16,14,14]}

            self.relu64_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_25_, output shape: {[1024,14,14]}

            self.batchnorm65_1_25_ = gluon.nn.BatchNorm()
            # batchnorm65_1_25_, output shape: {[1024,14,14]}

            self.conv63_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_26_, output shape: {[16,14,14]}

            self.batchnorm63_1_26_ = gluon.nn.BatchNorm()
            # batchnorm63_1_26_, output shape: {[16,14,14]}

            self.relu63_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_26_, output shape: {[16,14,14]}

            self.batchnorm64_1_26_ = gluon.nn.BatchNorm()
            # batchnorm64_1_26_, output shape: {[16,14,14]}

            self.relu64_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_26_, output shape: {[1024,14,14]}

            self.batchnorm65_1_26_ = gluon.nn.BatchNorm()
            # batchnorm65_1_26_, output shape: {[1024,14,14]}

            self.conv63_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_27_, output shape: {[16,14,14]}

            self.batchnorm63_1_27_ = gluon.nn.BatchNorm()
            # batchnorm63_1_27_, output shape: {[16,14,14]}

            self.relu63_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_27_, output shape: {[16,14,14]}

            self.batchnorm64_1_27_ = gluon.nn.BatchNorm()
            # batchnorm64_1_27_, output shape: {[16,14,14]}

            self.relu64_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_27_, output shape: {[1024,14,14]}

            self.batchnorm65_1_27_ = gluon.nn.BatchNorm()
            # batchnorm65_1_27_, output shape: {[1024,14,14]}

            self.conv63_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_28_, output shape: {[16,14,14]}

            self.batchnorm63_1_28_ = gluon.nn.BatchNorm()
            # batchnorm63_1_28_, output shape: {[16,14,14]}

            self.relu63_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_28_, output shape: {[16,14,14]}

            self.batchnorm64_1_28_ = gluon.nn.BatchNorm()
            # batchnorm64_1_28_, output shape: {[16,14,14]}

            self.relu64_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_28_, output shape: {[1024,14,14]}

            self.batchnorm65_1_28_ = gluon.nn.BatchNorm()
            # batchnorm65_1_28_, output shape: {[1024,14,14]}

            self.conv63_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_29_, output shape: {[16,14,14]}

            self.batchnorm63_1_29_ = gluon.nn.BatchNorm()
            # batchnorm63_1_29_, output shape: {[16,14,14]}

            self.relu63_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_29_, output shape: {[16,14,14]}

            self.batchnorm64_1_29_ = gluon.nn.BatchNorm()
            # batchnorm64_1_29_, output shape: {[16,14,14]}

            self.relu64_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_29_, output shape: {[1024,14,14]}

            self.batchnorm65_1_29_ = gluon.nn.BatchNorm()
            # batchnorm65_1_29_, output shape: {[1024,14,14]}

            self.conv63_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_30_, output shape: {[16,14,14]}

            self.batchnorm63_1_30_ = gluon.nn.BatchNorm()
            # batchnorm63_1_30_, output shape: {[16,14,14]}

            self.relu63_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_30_, output shape: {[16,14,14]}

            self.batchnorm64_1_30_ = gluon.nn.BatchNorm()
            # batchnorm64_1_30_, output shape: {[16,14,14]}

            self.relu64_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_30_, output shape: {[1024,14,14]}

            self.batchnorm65_1_30_ = gluon.nn.BatchNorm()
            # batchnorm65_1_30_, output shape: {[1024,14,14]}

            self.conv63_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_31_, output shape: {[16,14,14]}

            self.batchnorm63_1_31_ = gluon.nn.BatchNorm()
            # batchnorm63_1_31_, output shape: {[16,14,14]}

            self.relu63_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_31_, output shape: {[16,14,14]}

            self.batchnorm64_1_31_ = gluon.nn.BatchNorm()
            # batchnorm64_1_31_, output shape: {[16,14,14]}

            self.relu64_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_31_, output shape: {[1024,14,14]}

            self.batchnorm65_1_31_ = gluon.nn.BatchNorm()
            # batchnorm65_1_31_, output shape: {[1024,14,14]}

            self.conv63_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv63_1_32_, output shape: {[16,14,14]}

            self.batchnorm63_1_32_ = gluon.nn.BatchNorm()
            # batchnorm63_1_32_, output shape: {[16,14,14]}

            self.relu63_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv64_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv64_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv64_1_32_, output shape: {[16,14,14]}

            self.batchnorm64_1_32_ = gluon.nn.BatchNorm()
            # batchnorm64_1_32_, output shape: {[16,14,14]}

            self.relu64_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv65_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv65_1_32_, output shape: {[1024,14,14]}

            self.batchnorm65_1_32_ = gluon.nn.BatchNorm()
            # batchnorm65_1_32_, output shape: {[1024,14,14]}

            self.relu67_ = gluon.nn.Activation(activation='relu')
            self.conv69_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_1_, output shape: {[16,14,14]}

            self.batchnorm69_1_1_ = gluon.nn.BatchNorm()
            # batchnorm69_1_1_, output shape: {[16,14,14]}

            self.relu69_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_1_, output shape: {[16,14,14]}

            self.batchnorm70_1_1_ = gluon.nn.BatchNorm()
            # batchnorm70_1_1_, output shape: {[16,14,14]}

            self.relu70_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_1_, output shape: {[1024,14,14]}

            self.batchnorm71_1_1_ = gluon.nn.BatchNorm()
            # batchnorm71_1_1_, output shape: {[1024,14,14]}

            self.conv69_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_2_, output shape: {[16,14,14]}

            self.batchnorm69_1_2_ = gluon.nn.BatchNorm()
            # batchnorm69_1_2_, output shape: {[16,14,14]}

            self.relu69_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_2_, output shape: {[16,14,14]}

            self.batchnorm70_1_2_ = gluon.nn.BatchNorm()
            # batchnorm70_1_2_, output shape: {[16,14,14]}

            self.relu70_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_2_, output shape: {[1024,14,14]}

            self.batchnorm71_1_2_ = gluon.nn.BatchNorm()
            # batchnorm71_1_2_, output shape: {[1024,14,14]}

            self.conv69_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_3_, output shape: {[16,14,14]}

            self.batchnorm69_1_3_ = gluon.nn.BatchNorm()
            # batchnorm69_1_3_, output shape: {[16,14,14]}

            self.relu69_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_3_, output shape: {[16,14,14]}

            self.batchnorm70_1_3_ = gluon.nn.BatchNorm()
            # batchnorm70_1_3_, output shape: {[16,14,14]}

            self.relu70_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_3_, output shape: {[1024,14,14]}

            self.batchnorm71_1_3_ = gluon.nn.BatchNorm()
            # batchnorm71_1_3_, output shape: {[1024,14,14]}

            self.conv69_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_4_, output shape: {[16,14,14]}

            self.batchnorm69_1_4_ = gluon.nn.BatchNorm()
            # batchnorm69_1_4_, output shape: {[16,14,14]}

            self.relu69_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_4_, output shape: {[16,14,14]}

            self.batchnorm70_1_4_ = gluon.nn.BatchNorm()
            # batchnorm70_1_4_, output shape: {[16,14,14]}

            self.relu70_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_4_, output shape: {[1024,14,14]}

            self.batchnorm71_1_4_ = gluon.nn.BatchNorm()
            # batchnorm71_1_4_, output shape: {[1024,14,14]}

            self.conv69_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_5_, output shape: {[16,14,14]}

            self.batchnorm69_1_5_ = gluon.nn.BatchNorm()
            # batchnorm69_1_5_, output shape: {[16,14,14]}

            self.relu69_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_5_, output shape: {[16,14,14]}

            self.batchnorm70_1_5_ = gluon.nn.BatchNorm()
            # batchnorm70_1_5_, output shape: {[16,14,14]}

            self.relu70_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_5_, output shape: {[1024,14,14]}

            self.batchnorm71_1_5_ = gluon.nn.BatchNorm()
            # batchnorm71_1_5_, output shape: {[1024,14,14]}

            self.conv69_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_6_, output shape: {[16,14,14]}

            self.batchnorm69_1_6_ = gluon.nn.BatchNorm()
            # batchnorm69_1_6_, output shape: {[16,14,14]}

            self.relu69_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_6_, output shape: {[16,14,14]}

            self.batchnorm70_1_6_ = gluon.nn.BatchNorm()
            # batchnorm70_1_6_, output shape: {[16,14,14]}

            self.relu70_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_6_, output shape: {[1024,14,14]}

            self.batchnorm71_1_6_ = gluon.nn.BatchNorm()
            # batchnorm71_1_6_, output shape: {[1024,14,14]}

            self.conv69_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_7_, output shape: {[16,14,14]}

            self.batchnorm69_1_7_ = gluon.nn.BatchNorm()
            # batchnorm69_1_7_, output shape: {[16,14,14]}

            self.relu69_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_7_, output shape: {[16,14,14]}

            self.batchnorm70_1_7_ = gluon.nn.BatchNorm()
            # batchnorm70_1_7_, output shape: {[16,14,14]}

            self.relu70_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_7_, output shape: {[1024,14,14]}

            self.batchnorm71_1_7_ = gluon.nn.BatchNorm()
            # batchnorm71_1_7_, output shape: {[1024,14,14]}

            self.conv69_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_8_, output shape: {[16,14,14]}

            self.batchnorm69_1_8_ = gluon.nn.BatchNorm()
            # batchnorm69_1_8_, output shape: {[16,14,14]}

            self.relu69_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_8_, output shape: {[16,14,14]}

            self.batchnorm70_1_8_ = gluon.nn.BatchNorm()
            # batchnorm70_1_8_, output shape: {[16,14,14]}

            self.relu70_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_8_, output shape: {[1024,14,14]}

            self.batchnorm71_1_8_ = gluon.nn.BatchNorm()
            # batchnorm71_1_8_, output shape: {[1024,14,14]}

            self.conv69_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_9_, output shape: {[16,14,14]}

            self.batchnorm69_1_9_ = gluon.nn.BatchNorm()
            # batchnorm69_1_9_, output shape: {[16,14,14]}

            self.relu69_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_9_, output shape: {[16,14,14]}

            self.batchnorm70_1_9_ = gluon.nn.BatchNorm()
            # batchnorm70_1_9_, output shape: {[16,14,14]}

            self.relu70_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_9_, output shape: {[1024,14,14]}

            self.batchnorm71_1_9_ = gluon.nn.BatchNorm()
            # batchnorm71_1_9_, output shape: {[1024,14,14]}

            self.conv69_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_10_, output shape: {[16,14,14]}

            self.batchnorm69_1_10_ = gluon.nn.BatchNorm()
            # batchnorm69_1_10_, output shape: {[16,14,14]}

            self.relu69_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_10_, output shape: {[16,14,14]}

            self.batchnorm70_1_10_ = gluon.nn.BatchNorm()
            # batchnorm70_1_10_, output shape: {[16,14,14]}

            self.relu70_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_10_, output shape: {[1024,14,14]}

            self.batchnorm71_1_10_ = gluon.nn.BatchNorm()
            # batchnorm71_1_10_, output shape: {[1024,14,14]}

            self.conv69_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_11_, output shape: {[16,14,14]}

            self.batchnorm69_1_11_ = gluon.nn.BatchNorm()
            # batchnorm69_1_11_, output shape: {[16,14,14]}

            self.relu69_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_11_, output shape: {[16,14,14]}

            self.batchnorm70_1_11_ = gluon.nn.BatchNorm()
            # batchnorm70_1_11_, output shape: {[16,14,14]}

            self.relu70_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_11_, output shape: {[1024,14,14]}

            self.batchnorm71_1_11_ = gluon.nn.BatchNorm()
            # batchnorm71_1_11_, output shape: {[1024,14,14]}

            self.conv69_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_12_, output shape: {[16,14,14]}

            self.batchnorm69_1_12_ = gluon.nn.BatchNorm()
            # batchnorm69_1_12_, output shape: {[16,14,14]}

            self.relu69_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_12_, output shape: {[16,14,14]}

            self.batchnorm70_1_12_ = gluon.nn.BatchNorm()
            # batchnorm70_1_12_, output shape: {[16,14,14]}

            self.relu70_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_12_, output shape: {[1024,14,14]}

            self.batchnorm71_1_12_ = gluon.nn.BatchNorm()
            # batchnorm71_1_12_, output shape: {[1024,14,14]}

            self.conv69_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_13_, output shape: {[16,14,14]}

            self.batchnorm69_1_13_ = gluon.nn.BatchNorm()
            # batchnorm69_1_13_, output shape: {[16,14,14]}

            self.relu69_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_13_, output shape: {[16,14,14]}

            self.batchnorm70_1_13_ = gluon.nn.BatchNorm()
            # batchnorm70_1_13_, output shape: {[16,14,14]}

            self.relu70_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_13_, output shape: {[1024,14,14]}

            self.batchnorm71_1_13_ = gluon.nn.BatchNorm()
            # batchnorm71_1_13_, output shape: {[1024,14,14]}

            self.conv69_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_14_, output shape: {[16,14,14]}

            self.batchnorm69_1_14_ = gluon.nn.BatchNorm()
            # batchnorm69_1_14_, output shape: {[16,14,14]}

            self.relu69_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_14_, output shape: {[16,14,14]}

            self.batchnorm70_1_14_ = gluon.nn.BatchNorm()
            # batchnorm70_1_14_, output shape: {[16,14,14]}

            self.relu70_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_14_, output shape: {[1024,14,14]}

            self.batchnorm71_1_14_ = gluon.nn.BatchNorm()
            # batchnorm71_1_14_, output shape: {[1024,14,14]}

            self.conv69_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_15_, output shape: {[16,14,14]}

            self.batchnorm69_1_15_ = gluon.nn.BatchNorm()
            # batchnorm69_1_15_, output shape: {[16,14,14]}

            self.relu69_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_15_, output shape: {[16,14,14]}

            self.batchnorm70_1_15_ = gluon.nn.BatchNorm()
            # batchnorm70_1_15_, output shape: {[16,14,14]}

            self.relu70_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_15_, output shape: {[1024,14,14]}

            self.batchnorm71_1_15_ = gluon.nn.BatchNorm()
            # batchnorm71_1_15_, output shape: {[1024,14,14]}

            self.conv69_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_16_, output shape: {[16,14,14]}

            self.batchnorm69_1_16_ = gluon.nn.BatchNorm()
            # batchnorm69_1_16_, output shape: {[16,14,14]}

            self.relu69_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_16_, output shape: {[16,14,14]}

            self.batchnorm70_1_16_ = gluon.nn.BatchNorm()
            # batchnorm70_1_16_, output shape: {[16,14,14]}

            self.relu70_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_16_, output shape: {[1024,14,14]}

            self.batchnorm71_1_16_ = gluon.nn.BatchNorm()
            # batchnorm71_1_16_, output shape: {[1024,14,14]}

            self.conv69_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_17_, output shape: {[16,14,14]}

            self.batchnorm69_1_17_ = gluon.nn.BatchNorm()
            # batchnorm69_1_17_, output shape: {[16,14,14]}

            self.relu69_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_17_, output shape: {[16,14,14]}

            self.batchnorm70_1_17_ = gluon.nn.BatchNorm()
            # batchnorm70_1_17_, output shape: {[16,14,14]}

            self.relu70_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_17_, output shape: {[1024,14,14]}

            self.batchnorm71_1_17_ = gluon.nn.BatchNorm()
            # batchnorm71_1_17_, output shape: {[1024,14,14]}

            self.conv69_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_18_, output shape: {[16,14,14]}

            self.batchnorm69_1_18_ = gluon.nn.BatchNorm()
            # batchnorm69_1_18_, output shape: {[16,14,14]}

            self.relu69_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_18_, output shape: {[16,14,14]}

            self.batchnorm70_1_18_ = gluon.nn.BatchNorm()
            # batchnorm70_1_18_, output shape: {[16,14,14]}

            self.relu70_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_18_, output shape: {[1024,14,14]}

            self.batchnorm71_1_18_ = gluon.nn.BatchNorm()
            # batchnorm71_1_18_, output shape: {[1024,14,14]}

            self.conv69_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_19_, output shape: {[16,14,14]}

            self.batchnorm69_1_19_ = gluon.nn.BatchNorm()
            # batchnorm69_1_19_, output shape: {[16,14,14]}

            self.relu69_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_19_, output shape: {[16,14,14]}

            self.batchnorm70_1_19_ = gluon.nn.BatchNorm()
            # batchnorm70_1_19_, output shape: {[16,14,14]}

            self.relu70_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_19_, output shape: {[1024,14,14]}

            self.batchnorm71_1_19_ = gluon.nn.BatchNorm()
            # batchnorm71_1_19_, output shape: {[1024,14,14]}

            self.conv69_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_20_, output shape: {[16,14,14]}

            self.batchnorm69_1_20_ = gluon.nn.BatchNorm()
            # batchnorm69_1_20_, output shape: {[16,14,14]}

            self.relu69_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_20_, output shape: {[16,14,14]}

            self.batchnorm70_1_20_ = gluon.nn.BatchNorm()
            # batchnorm70_1_20_, output shape: {[16,14,14]}

            self.relu70_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_20_, output shape: {[1024,14,14]}

            self.batchnorm71_1_20_ = gluon.nn.BatchNorm()
            # batchnorm71_1_20_, output shape: {[1024,14,14]}

            self.conv69_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_21_, output shape: {[16,14,14]}

            self.batchnorm69_1_21_ = gluon.nn.BatchNorm()
            # batchnorm69_1_21_, output shape: {[16,14,14]}

            self.relu69_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_21_, output shape: {[16,14,14]}

            self.batchnorm70_1_21_ = gluon.nn.BatchNorm()
            # batchnorm70_1_21_, output shape: {[16,14,14]}

            self.relu70_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_21_, output shape: {[1024,14,14]}

            self.batchnorm71_1_21_ = gluon.nn.BatchNorm()
            # batchnorm71_1_21_, output shape: {[1024,14,14]}

            self.conv69_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_22_, output shape: {[16,14,14]}

            self.batchnorm69_1_22_ = gluon.nn.BatchNorm()
            # batchnorm69_1_22_, output shape: {[16,14,14]}

            self.relu69_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_22_, output shape: {[16,14,14]}

            self.batchnorm70_1_22_ = gluon.nn.BatchNorm()
            # batchnorm70_1_22_, output shape: {[16,14,14]}

            self.relu70_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_22_, output shape: {[1024,14,14]}

            self.batchnorm71_1_22_ = gluon.nn.BatchNorm()
            # batchnorm71_1_22_, output shape: {[1024,14,14]}

            self.conv69_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_23_, output shape: {[16,14,14]}

            self.batchnorm69_1_23_ = gluon.nn.BatchNorm()
            # batchnorm69_1_23_, output shape: {[16,14,14]}

            self.relu69_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_23_, output shape: {[16,14,14]}

            self.batchnorm70_1_23_ = gluon.nn.BatchNorm()
            # batchnorm70_1_23_, output shape: {[16,14,14]}

            self.relu70_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_23_, output shape: {[1024,14,14]}

            self.batchnorm71_1_23_ = gluon.nn.BatchNorm()
            # batchnorm71_1_23_, output shape: {[1024,14,14]}

            self.conv69_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_24_, output shape: {[16,14,14]}

            self.batchnorm69_1_24_ = gluon.nn.BatchNorm()
            # batchnorm69_1_24_, output shape: {[16,14,14]}

            self.relu69_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_24_, output shape: {[16,14,14]}

            self.batchnorm70_1_24_ = gluon.nn.BatchNorm()
            # batchnorm70_1_24_, output shape: {[16,14,14]}

            self.relu70_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_24_, output shape: {[1024,14,14]}

            self.batchnorm71_1_24_ = gluon.nn.BatchNorm()
            # batchnorm71_1_24_, output shape: {[1024,14,14]}

            self.conv69_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_25_, output shape: {[16,14,14]}

            self.batchnorm69_1_25_ = gluon.nn.BatchNorm()
            # batchnorm69_1_25_, output shape: {[16,14,14]}

            self.relu69_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_25_, output shape: {[16,14,14]}

            self.batchnorm70_1_25_ = gluon.nn.BatchNorm()
            # batchnorm70_1_25_, output shape: {[16,14,14]}

            self.relu70_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_25_, output shape: {[1024,14,14]}

            self.batchnorm71_1_25_ = gluon.nn.BatchNorm()
            # batchnorm71_1_25_, output shape: {[1024,14,14]}

            self.conv69_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_26_, output shape: {[16,14,14]}

            self.batchnorm69_1_26_ = gluon.nn.BatchNorm()
            # batchnorm69_1_26_, output shape: {[16,14,14]}

            self.relu69_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_26_, output shape: {[16,14,14]}

            self.batchnorm70_1_26_ = gluon.nn.BatchNorm()
            # batchnorm70_1_26_, output shape: {[16,14,14]}

            self.relu70_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_26_, output shape: {[1024,14,14]}

            self.batchnorm71_1_26_ = gluon.nn.BatchNorm()
            # batchnorm71_1_26_, output shape: {[1024,14,14]}

            self.conv69_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_27_, output shape: {[16,14,14]}

            self.batchnorm69_1_27_ = gluon.nn.BatchNorm()
            # batchnorm69_1_27_, output shape: {[16,14,14]}

            self.relu69_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_27_, output shape: {[16,14,14]}

            self.batchnorm70_1_27_ = gluon.nn.BatchNorm()
            # batchnorm70_1_27_, output shape: {[16,14,14]}

            self.relu70_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_27_, output shape: {[1024,14,14]}

            self.batchnorm71_1_27_ = gluon.nn.BatchNorm()
            # batchnorm71_1_27_, output shape: {[1024,14,14]}

            self.conv69_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_28_, output shape: {[16,14,14]}

            self.batchnorm69_1_28_ = gluon.nn.BatchNorm()
            # batchnorm69_1_28_, output shape: {[16,14,14]}

            self.relu69_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_28_, output shape: {[16,14,14]}

            self.batchnorm70_1_28_ = gluon.nn.BatchNorm()
            # batchnorm70_1_28_, output shape: {[16,14,14]}

            self.relu70_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_28_, output shape: {[1024,14,14]}

            self.batchnorm71_1_28_ = gluon.nn.BatchNorm()
            # batchnorm71_1_28_, output shape: {[1024,14,14]}

            self.conv69_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_29_, output shape: {[16,14,14]}

            self.batchnorm69_1_29_ = gluon.nn.BatchNorm()
            # batchnorm69_1_29_, output shape: {[16,14,14]}

            self.relu69_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_29_, output shape: {[16,14,14]}

            self.batchnorm70_1_29_ = gluon.nn.BatchNorm()
            # batchnorm70_1_29_, output shape: {[16,14,14]}

            self.relu70_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_29_, output shape: {[1024,14,14]}

            self.batchnorm71_1_29_ = gluon.nn.BatchNorm()
            # batchnorm71_1_29_, output shape: {[1024,14,14]}

            self.conv69_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_30_, output shape: {[16,14,14]}

            self.batchnorm69_1_30_ = gluon.nn.BatchNorm()
            # batchnorm69_1_30_, output shape: {[16,14,14]}

            self.relu69_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_30_, output shape: {[16,14,14]}

            self.batchnorm70_1_30_ = gluon.nn.BatchNorm()
            # batchnorm70_1_30_, output shape: {[16,14,14]}

            self.relu70_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_30_, output shape: {[1024,14,14]}

            self.batchnorm71_1_30_ = gluon.nn.BatchNorm()
            # batchnorm71_1_30_, output shape: {[1024,14,14]}

            self.conv69_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_31_, output shape: {[16,14,14]}

            self.batchnorm69_1_31_ = gluon.nn.BatchNorm()
            # batchnorm69_1_31_, output shape: {[16,14,14]}

            self.relu69_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_31_, output shape: {[16,14,14]}

            self.batchnorm70_1_31_ = gluon.nn.BatchNorm()
            # batchnorm70_1_31_, output shape: {[16,14,14]}

            self.relu70_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_31_, output shape: {[1024,14,14]}

            self.batchnorm71_1_31_ = gluon.nn.BatchNorm()
            # batchnorm71_1_31_, output shape: {[1024,14,14]}

            self.conv69_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv69_1_32_, output shape: {[16,14,14]}

            self.batchnorm69_1_32_ = gluon.nn.BatchNorm()
            # batchnorm69_1_32_, output shape: {[16,14,14]}

            self.relu69_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv70_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv70_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv70_1_32_, output shape: {[16,14,14]}

            self.batchnorm70_1_32_ = gluon.nn.BatchNorm()
            # batchnorm70_1_32_, output shape: {[16,14,14]}

            self.relu70_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv71_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv71_1_32_, output shape: {[1024,14,14]}

            self.batchnorm71_1_32_ = gluon.nn.BatchNorm()
            # batchnorm71_1_32_, output shape: {[1024,14,14]}

            self.relu73_ = gluon.nn.Activation(activation='relu')
            self.conv75_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_1_, output shape: {[16,14,14]}

            self.batchnorm75_1_1_ = gluon.nn.BatchNorm()
            # batchnorm75_1_1_, output shape: {[16,14,14]}

            self.relu75_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_1_, output shape: {[16,14,14]}

            self.batchnorm76_1_1_ = gluon.nn.BatchNorm()
            # batchnorm76_1_1_, output shape: {[16,14,14]}

            self.relu76_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_1_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_1_, output shape: {[1024,14,14]}

            self.batchnorm77_1_1_ = gluon.nn.BatchNorm()
            # batchnorm77_1_1_, output shape: {[1024,14,14]}

            self.conv75_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_2_, output shape: {[16,14,14]}

            self.batchnorm75_1_2_ = gluon.nn.BatchNorm()
            # batchnorm75_1_2_, output shape: {[16,14,14]}

            self.relu75_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_2_, output shape: {[16,14,14]}

            self.batchnorm76_1_2_ = gluon.nn.BatchNorm()
            # batchnorm76_1_2_, output shape: {[16,14,14]}

            self.relu76_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_2_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_2_, output shape: {[1024,14,14]}

            self.batchnorm77_1_2_ = gluon.nn.BatchNorm()
            # batchnorm77_1_2_, output shape: {[1024,14,14]}

            self.conv75_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_3_, output shape: {[16,14,14]}

            self.batchnorm75_1_3_ = gluon.nn.BatchNorm()
            # batchnorm75_1_3_, output shape: {[16,14,14]}

            self.relu75_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_3_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_3_, output shape: {[16,14,14]}

            self.batchnorm76_1_3_ = gluon.nn.BatchNorm()
            # batchnorm76_1_3_, output shape: {[16,14,14]}

            self.relu76_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_3_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_3_, output shape: {[1024,14,14]}

            self.batchnorm77_1_3_ = gluon.nn.BatchNorm()
            # batchnorm77_1_3_, output shape: {[1024,14,14]}

            self.conv75_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_4_, output shape: {[16,14,14]}

            self.batchnorm75_1_4_ = gluon.nn.BatchNorm()
            # batchnorm75_1_4_, output shape: {[16,14,14]}

            self.relu75_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_4_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_4_, output shape: {[16,14,14]}

            self.batchnorm76_1_4_ = gluon.nn.BatchNorm()
            # batchnorm76_1_4_, output shape: {[16,14,14]}

            self.relu76_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_4_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_4_, output shape: {[1024,14,14]}

            self.batchnorm77_1_4_ = gluon.nn.BatchNorm()
            # batchnorm77_1_4_, output shape: {[1024,14,14]}

            self.conv75_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_5_, output shape: {[16,14,14]}

            self.batchnorm75_1_5_ = gluon.nn.BatchNorm()
            # batchnorm75_1_5_, output shape: {[16,14,14]}

            self.relu75_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_5_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_5_, output shape: {[16,14,14]}

            self.batchnorm76_1_5_ = gluon.nn.BatchNorm()
            # batchnorm76_1_5_, output shape: {[16,14,14]}

            self.relu76_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_5_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_5_, output shape: {[1024,14,14]}

            self.batchnorm77_1_5_ = gluon.nn.BatchNorm()
            # batchnorm77_1_5_, output shape: {[1024,14,14]}

            self.conv75_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_6_, output shape: {[16,14,14]}

            self.batchnorm75_1_6_ = gluon.nn.BatchNorm()
            # batchnorm75_1_6_, output shape: {[16,14,14]}

            self.relu75_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_6_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_6_, output shape: {[16,14,14]}

            self.batchnorm76_1_6_ = gluon.nn.BatchNorm()
            # batchnorm76_1_6_, output shape: {[16,14,14]}

            self.relu76_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_6_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_6_, output shape: {[1024,14,14]}

            self.batchnorm77_1_6_ = gluon.nn.BatchNorm()
            # batchnorm77_1_6_, output shape: {[1024,14,14]}

            self.conv75_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_7_, output shape: {[16,14,14]}

            self.batchnorm75_1_7_ = gluon.nn.BatchNorm()
            # batchnorm75_1_7_, output shape: {[16,14,14]}

            self.relu75_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_7_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_7_, output shape: {[16,14,14]}

            self.batchnorm76_1_7_ = gluon.nn.BatchNorm()
            # batchnorm76_1_7_, output shape: {[16,14,14]}

            self.relu76_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_7_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_7_, output shape: {[1024,14,14]}

            self.batchnorm77_1_7_ = gluon.nn.BatchNorm()
            # batchnorm77_1_7_, output shape: {[1024,14,14]}

            self.conv75_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_8_, output shape: {[16,14,14]}

            self.batchnorm75_1_8_ = gluon.nn.BatchNorm()
            # batchnorm75_1_8_, output shape: {[16,14,14]}

            self.relu75_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_8_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_8_, output shape: {[16,14,14]}

            self.batchnorm76_1_8_ = gluon.nn.BatchNorm()
            # batchnorm76_1_8_, output shape: {[16,14,14]}

            self.relu76_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_8_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_8_, output shape: {[1024,14,14]}

            self.batchnorm77_1_8_ = gluon.nn.BatchNorm()
            # batchnorm77_1_8_, output shape: {[1024,14,14]}

            self.conv75_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_9_, output shape: {[16,14,14]}

            self.batchnorm75_1_9_ = gluon.nn.BatchNorm()
            # batchnorm75_1_9_, output shape: {[16,14,14]}

            self.relu75_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_9_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_9_, output shape: {[16,14,14]}

            self.batchnorm76_1_9_ = gluon.nn.BatchNorm()
            # batchnorm76_1_9_, output shape: {[16,14,14]}

            self.relu76_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_9_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_9_, output shape: {[1024,14,14]}

            self.batchnorm77_1_9_ = gluon.nn.BatchNorm()
            # batchnorm77_1_9_, output shape: {[1024,14,14]}

            self.conv75_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_10_, output shape: {[16,14,14]}

            self.batchnorm75_1_10_ = gluon.nn.BatchNorm()
            # batchnorm75_1_10_, output shape: {[16,14,14]}

            self.relu75_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_10_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_10_, output shape: {[16,14,14]}

            self.batchnorm76_1_10_ = gluon.nn.BatchNorm()
            # batchnorm76_1_10_, output shape: {[16,14,14]}

            self.relu76_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_10_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_10_, output shape: {[1024,14,14]}

            self.batchnorm77_1_10_ = gluon.nn.BatchNorm()
            # batchnorm77_1_10_, output shape: {[1024,14,14]}

            self.conv75_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_11_, output shape: {[16,14,14]}

            self.batchnorm75_1_11_ = gluon.nn.BatchNorm()
            # batchnorm75_1_11_, output shape: {[16,14,14]}

            self.relu75_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_11_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_11_, output shape: {[16,14,14]}

            self.batchnorm76_1_11_ = gluon.nn.BatchNorm()
            # batchnorm76_1_11_, output shape: {[16,14,14]}

            self.relu76_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_11_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_11_, output shape: {[1024,14,14]}

            self.batchnorm77_1_11_ = gluon.nn.BatchNorm()
            # batchnorm77_1_11_, output shape: {[1024,14,14]}

            self.conv75_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_12_, output shape: {[16,14,14]}

            self.batchnorm75_1_12_ = gluon.nn.BatchNorm()
            # batchnorm75_1_12_, output shape: {[16,14,14]}

            self.relu75_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_12_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_12_, output shape: {[16,14,14]}

            self.batchnorm76_1_12_ = gluon.nn.BatchNorm()
            # batchnorm76_1_12_, output shape: {[16,14,14]}

            self.relu76_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_12_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_12_, output shape: {[1024,14,14]}

            self.batchnorm77_1_12_ = gluon.nn.BatchNorm()
            # batchnorm77_1_12_, output shape: {[1024,14,14]}

            self.conv75_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_13_, output shape: {[16,14,14]}

            self.batchnorm75_1_13_ = gluon.nn.BatchNorm()
            # batchnorm75_1_13_, output shape: {[16,14,14]}

            self.relu75_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_13_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_13_, output shape: {[16,14,14]}

            self.batchnorm76_1_13_ = gluon.nn.BatchNorm()
            # batchnorm76_1_13_, output shape: {[16,14,14]}

            self.relu76_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_13_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_13_, output shape: {[1024,14,14]}

            self.batchnorm77_1_13_ = gluon.nn.BatchNorm()
            # batchnorm77_1_13_, output shape: {[1024,14,14]}

            self.conv75_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_14_, output shape: {[16,14,14]}

            self.batchnorm75_1_14_ = gluon.nn.BatchNorm()
            # batchnorm75_1_14_, output shape: {[16,14,14]}

            self.relu75_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_14_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_14_, output shape: {[16,14,14]}

            self.batchnorm76_1_14_ = gluon.nn.BatchNorm()
            # batchnorm76_1_14_, output shape: {[16,14,14]}

            self.relu76_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_14_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_14_, output shape: {[1024,14,14]}

            self.batchnorm77_1_14_ = gluon.nn.BatchNorm()
            # batchnorm77_1_14_, output shape: {[1024,14,14]}

            self.conv75_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_15_, output shape: {[16,14,14]}

            self.batchnorm75_1_15_ = gluon.nn.BatchNorm()
            # batchnorm75_1_15_, output shape: {[16,14,14]}

            self.relu75_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_15_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_15_, output shape: {[16,14,14]}

            self.batchnorm76_1_15_ = gluon.nn.BatchNorm()
            # batchnorm76_1_15_, output shape: {[16,14,14]}

            self.relu76_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_15_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_15_, output shape: {[1024,14,14]}

            self.batchnorm77_1_15_ = gluon.nn.BatchNorm()
            # batchnorm77_1_15_, output shape: {[1024,14,14]}

            self.conv75_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_16_, output shape: {[16,14,14]}

            self.batchnorm75_1_16_ = gluon.nn.BatchNorm()
            # batchnorm75_1_16_, output shape: {[16,14,14]}

            self.relu75_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_16_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_16_, output shape: {[16,14,14]}

            self.batchnorm76_1_16_ = gluon.nn.BatchNorm()
            # batchnorm76_1_16_, output shape: {[16,14,14]}

            self.relu76_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_16_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_16_, output shape: {[1024,14,14]}

            self.batchnorm77_1_16_ = gluon.nn.BatchNorm()
            # batchnorm77_1_16_, output shape: {[1024,14,14]}

            self.conv75_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_17_, output shape: {[16,14,14]}

            self.batchnorm75_1_17_ = gluon.nn.BatchNorm()
            # batchnorm75_1_17_, output shape: {[16,14,14]}

            self.relu75_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_17_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_17_, output shape: {[16,14,14]}

            self.batchnorm76_1_17_ = gluon.nn.BatchNorm()
            # batchnorm76_1_17_, output shape: {[16,14,14]}

            self.relu76_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_17_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_17_, output shape: {[1024,14,14]}

            self.batchnorm77_1_17_ = gluon.nn.BatchNorm()
            # batchnorm77_1_17_, output shape: {[1024,14,14]}

            self.conv75_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_18_, output shape: {[16,14,14]}

            self.batchnorm75_1_18_ = gluon.nn.BatchNorm()
            # batchnorm75_1_18_, output shape: {[16,14,14]}

            self.relu75_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_18_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_18_, output shape: {[16,14,14]}

            self.batchnorm76_1_18_ = gluon.nn.BatchNorm()
            # batchnorm76_1_18_, output shape: {[16,14,14]}

            self.relu76_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_18_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_18_, output shape: {[1024,14,14]}

            self.batchnorm77_1_18_ = gluon.nn.BatchNorm()
            # batchnorm77_1_18_, output shape: {[1024,14,14]}

            self.conv75_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_19_, output shape: {[16,14,14]}

            self.batchnorm75_1_19_ = gluon.nn.BatchNorm()
            # batchnorm75_1_19_, output shape: {[16,14,14]}

            self.relu75_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_19_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_19_, output shape: {[16,14,14]}

            self.batchnorm76_1_19_ = gluon.nn.BatchNorm()
            # batchnorm76_1_19_, output shape: {[16,14,14]}

            self.relu76_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_19_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_19_, output shape: {[1024,14,14]}

            self.batchnorm77_1_19_ = gluon.nn.BatchNorm()
            # batchnorm77_1_19_, output shape: {[1024,14,14]}

            self.conv75_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_20_, output shape: {[16,14,14]}

            self.batchnorm75_1_20_ = gluon.nn.BatchNorm()
            # batchnorm75_1_20_, output shape: {[16,14,14]}

            self.relu75_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_20_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_20_, output shape: {[16,14,14]}

            self.batchnorm76_1_20_ = gluon.nn.BatchNorm()
            # batchnorm76_1_20_, output shape: {[16,14,14]}

            self.relu76_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_20_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_20_, output shape: {[1024,14,14]}

            self.batchnorm77_1_20_ = gluon.nn.BatchNorm()
            # batchnorm77_1_20_, output shape: {[1024,14,14]}

            self.conv75_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_21_, output shape: {[16,14,14]}

            self.batchnorm75_1_21_ = gluon.nn.BatchNorm()
            # batchnorm75_1_21_, output shape: {[16,14,14]}

            self.relu75_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_21_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_21_, output shape: {[16,14,14]}

            self.batchnorm76_1_21_ = gluon.nn.BatchNorm()
            # batchnorm76_1_21_, output shape: {[16,14,14]}

            self.relu76_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_21_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_21_, output shape: {[1024,14,14]}

            self.batchnorm77_1_21_ = gluon.nn.BatchNorm()
            # batchnorm77_1_21_, output shape: {[1024,14,14]}

            self.conv75_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_22_, output shape: {[16,14,14]}

            self.batchnorm75_1_22_ = gluon.nn.BatchNorm()
            # batchnorm75_1_22_, output shape: {[16,14,14]}

            self.relu75_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_22_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_22_, output shape: {[16,14,14]}

            self.batchnorm76_1_22_ = gluon.nn.BatchNorm()
            # batchnorm76_1_22_, output shape: {[16,14,14]}

            self.relu76_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_22_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_22_, output shape: {[1024,14,14]}

            self.batchnorm77_1_22_ = gluon.nn.BatchNorm()
            # batchnorm77_1_22_, output shape: {[1024,14,14]}

            self.conv75_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_23_, output shape: {[16,14,14]}

            self.batchnorm75_1_23_ = gluon.nn.BatchNorm()
            # batchnorm75_1_23_, output shape: {[16,14,14]}

            self.relu75_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_23_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_23_, output shape: {[16,14,14]}

            self.batchnorm76_1_23_ = gluon.nn.BatchNorm()
            # batchnorm76_1_23_, output shape: {[16,14,14]}

            self.relu76_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_23_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_23_, output shape: {[1024,14,14]}

            self.batchnorm77_1_23_ = gluon.nn.BatchNorm()
            # batchnorm77_1_23_, output shape: {[1024,14,14]}

            self.conv75_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_24_, output shape: {[16,14,14]}

            self.batchnorm75_1_24_ = gluon.nn.BatchNorm()
            # batchnorm75_1_24_, output shape: {[16,14,14]}

            self.relu75_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_24_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_24_, output shape: {[16,14,14]}

            self.batchnorm76_1_24_ = gluon.nn.BatchNorm()
            # batchnorm76_1_24_, output shape: {[16,14,14]}

            self.relu76_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_24_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_24_, output shape: {[1024,14,14]}

            self.batchnorm77_1_24_ = gluon.nn.BatchNorm()
            # batchnorm77_1_24_, output shape: {[1024,14,14]}

            self.conv75_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_25_, output shape: {[16,14,14]}

            self.batchnorm75_1_25_ = gluon.nn.BatchNorm()
            # batchnorm75_1_25_, output shape: {[16,14,14]}

            self.relu75_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_25_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_25_, output shape: {[16,14,14]}

            self.batchnorm76_1_25_ = gluon.nn.BatchNorm()
            # batchnorm76_1_25_, output shape: {[16,14,14]}

            self.relu76_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_25_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_25_, output shape: {[1024,14,14]}

            self.batchnorm77_1_25_ = gluon.nn.BatchNorm()
            # batchnorm77_1_25_, output shape: {[1024,14,14]}

            self.conv75_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_26_, output shape: {[16,14,14]}

            self.batchnorm75_1_26_ = gluon.nn.BatchNorm()
            # batchnorm75_1_26_, output shape: {[16,14,14]}

            self.relu75_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_26_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_26_, output shape: {[16,14,14]}

            self.batchnorm76_1_26_ = gluon.nn.BatchNorm()
            # batchnorm76_1_26_, output shape: {[16,14,14]}

            self.relu76_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_26_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_26_, output shape: {[1024,14,14]}

            self.batchnorm77_1_26_ = gluon.nn.BatchNorm()
            # batchnorm77_1_26_, output shape: {[1024,14,14]}

            self.conv75_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_27_, output shape: {[16,14,14]}

            self.batchnorm75_1_27_ = gluon.nn.BatchNorm()
            # batchnorm75_1_27_, output shape: {[16,14,14]}

            self.relu75_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_27_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_27_, output shape: {[16,14,14]}

            self.batchnorm76_1_27_ = gluon.nn.BatchNorm()
            # batchnorm76_1_27_, output shape: {[16,14,14]}

            self.relu76_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_27_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_27_, output shape: {[1024,14,14]}

            self.batchnorm77_1_27_ = gluon.nn.BatchNorm()
            # batchnorm77_1_27_, output shape: {[1024,14,14]}

            self.conv75_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_28_, output shape: {[16,14,14]}

            self.batchnorm75_1_28_ = gluon.nn.BatchNorm()
            # batchnorm75_1_28_, output shape: {[16,14,14]}

            self.relu75_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_28_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_28_, output shape: {[16,14,14]}

            self.batchnorm76_1_28_ = gluon.nn.BatchNorm()
            # batchnorm76_1_28_, output shape: {[16,14,14]}

            self.relu76_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_28_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_28_, output shape: {[1024,14,14]}

            self.batchnorm77_1_28_ = gluon.nn.BatchNorm()
            # batchnorm77_1_28_, output shape: {[1024,14,14]}

            self.conv75_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_29_, output shape: {[16,14,14]}

            self.batchnorm75_1_29_ = gluon.nn.BatchNorm()
            # batchnorm75_1_29_, output shape: {[16,14,14]}

            self.relu75_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_29_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_29_, output shape: {[16,14,14]}

            self.batchnorm76_1_29_ = gluon.nn.BatchNorm()
            # batchnorm76_1_29_, output shape: {[16,14,14]}

            self.relu76_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_29_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_29_, output shape: {[1024,14,14]}

            self.batchnorm77_1_29_ = gluon.nn.BatchNorm()
            # batchnorm77_1_29_, output shape: {[1024,14,14]}

            self.conv75_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_30_, output shape: {[16,14,14]}

            self.batchnorm75_1_30_ = gluon.nn.BatchNorm()
            # batchnorm75_1_30_, output shape: {[16,14,14]}

            self.relu75_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_30_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_30_, output shape: {[16,14,14]}

            self.batchnorm76_1_30_ = gluon.nn.BatchNorm()
            # batchnorm76_1_30_, output shape: {[16,14,14]}

            self.relu76_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_30_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_30_, output shape: {[1024,14,14]}

            self.batchnorm77_1_30_ = gluon.nn.BatchNorm()
            # batchnorm77_1_30_, output shape: {[1024,14,14]}

            self.conv75_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_31_, output shape: {[16,14,14]}

            self.batchnorm75_1_31_ = gluon.nn.BatchNorm()
            # batchnorm75_1_31_, output shape: {[16,14,14]}

            self.relu75_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_31_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_31_, output shape: {[16,14,14]}

            self.batchnorm76_1_31_ = gluon.nn.BatchNorm()
            # batchnorm76_1_31_, output shape: {[16,14,14]}

            self.relu76_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_31_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_31_, output shape: {[1024,14,14]}

            self.batchnorm77_1_31_ = gluon.nn.BatchNorm()
            # batchnorm77_1_31_, output shape: {[1024,14,14]}

            self.conv75_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv75_1_32_, output shape: {[16,14,14]}

            self.batchnorm75_1_32_ = gluon.nn.BatchNorm()
            # batchnorm75_1_32_, output shape: {[16,14,14]}

            self.relu75_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv76_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv76_1_32_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv76_1_32_, output shape: {[16,14,14]}

            self.batchnorm76_1_32_ = gluon.nn.BatchNorm()
            # batchnorm76_1_32_, output shape: {[16,14,14]}

            self.relu76_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv77_1_32_ = gluon.nn.Conv2D(channels=1024,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv77_1_32_, output shape: {[1024,14,14]}

            self.batchnorm77_1_32_ = gluon.nn.BatchNorm()
            # batchnorm77_1_32_, output shape: {[1024,14,14]}

            self.relu79_ = gluon.nn.Activation(activation='relu')
            self.conv81_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_1_, output shape: {[32,14,14]}

            self.batchnorm81_1_1_ = gluon.nn.BatchNorm()
            # batchnorm81_1_1_, output shape: {[32,14,14]}

            self.relu81_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_1_, output shape: {[32,7,7]}

            self.batchnorm82_1_1_ = gluon.nn.BatchNorm()
            # batchnorm82_1_1_, output shape: {[32,7,7]}

            self.relu82_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_1_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_1_, output shape: {[2048,7,7]}

            self.batchnorm83_1_1_ = gluon.nn.BatchNorm()
            # batchnorm83_1_1_, output shape: {[2048,7,7]}

            self.conv81_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_2_, output shape: {[32,14,14]}

            self.batchnorm81_1_2_ = gluon.nn.BatchNorm()
            # batchnorm81_1_2_, output shape: {[32,14,14]}

            self.relu81_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_2_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_2_, output shape: {[32,7,7]}

            self.batchnorm82_1_2_ = gluon.nn.BatchNorm()
            # batchnorm82_1_2_, output shape: {[32,7,7]}

            self.relu82_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_2_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_2_, output shape: {[2048,7,7]}

            self.batchnorm83_1_2_ = gluon.nn.BatchNorm()
            # batchnorm83_1_2_, output shape: {[2048,7,7]}

            self.conv81_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_3_, output shape: {[32,14,14]}

            self.batchnorm81_1_3_ = gluon.nn.BatchNorm()
            # batchnorm81_1_3_, output shape: {[32,14,14]}

            self.relu81_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_3_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_3_, output shape: {[32,7,7]}

            self.batchnorm82_1_3_ = gluon.nn.BatchNorm()
            # batchnorm82_1_3_, output shape: {[32,7,7]}

            self.relu82_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_3_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_3_, output shape: {[2048,7,7]}

            self.batchnorm83_1_3_ = gluon.nn.BatchNorm()
            # batchnorm83_1_3_, output shape: {[2048,7,7]}

            self.conv81_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_4_, output shape: {[32,14,14]}

            self.batchnorm81_1_4_ = gluon.nn.BatchNorm()
            # batchnorm81_1_4_, output shape: {[32,14,14]}

            self.relu81_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_4_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_4_, output shape: {[32,7,7]}

            self.batchnorm82_1_4_ = gluon.nn.BatchNorm()
            # batchnorm82_1_4_, output shape: {[32,7,7]}

            self.relu82_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_4_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_4_, output shape: {[2048,7,7]}

            self.batchnorm83_1_4_ = gluon.nn.BatchNorm()
            # batchnorm83_1_4_, output shape: {[2048,7,7]}

            self.conv81_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_5_, output shape: {[32,14,14]}

            self.batchnorm81_1_5_ = gluon.nn.BatchNorm()
            # batchnorm81_1_5_, output shape: {[32,14,14]}

            self.relu81_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_5_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_5_, output shape: {[32,7,7]}

            self.batchnorm82_1_5_ = gluon.nn.BatchNorm()
            # batchnorm82_1_5_, output shape: {[32,7,7]}

            self.relu82_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_5_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_5_, output shape: {[2048,7,7]}

            self.batchnorm83_1_5_ = gluon.nn.BatchNorm()
            # batchnorm83_1_5_, output shape: {[2048,7,7]}

            self.conv81_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_6_, output shape: {[32,14,14]}

            self.batchnorm81_1_6_ = gluon.nn.BatchNorm()
            # batchnorm81_1_6_, output shape: {[32,14,14]}

            self.relu81_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_6_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_6_, output shape: {[32,7,7]}

            self.batchnorm82_1_6_ = gluon.nn.BatchNorm()
            # batchnorm82_1_6_, output shape: {[32,7,7]}

            self.relu82_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_6_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_6_, output shape: {[2048,7,7]}

            self.batchnorm83_1_6_ = gluon.nn.BatchNorm()
            # batchnorm83_1_6_, output shape: {[2048,7,7]}

            self.conv81_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_7_, output shape: {[32,14,14]}

            self.batchnorm81_1_7_ = gluon.nn.BatchNorm()
            # batchnorm81_1_7_, output shape: {[32,14,14]}

            self.relu81_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_7_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_7_, output shape: {[32,7,7]}

            self.batchnorm82_1_7_ = gluon.nn.BatchNorm()
            # batchnorm82_1_7_, output shape: {[32,7,7]}

            self.relu82_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_7_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_7_, output shape: {[2048,7,7]}

            self.batchnorm83_1_7_ = gluon.nn.BatchNorm()
            # batchnorm83_1_7_, output shape: {[2048,7,7]}

            self.conv81_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_8_, output shape: {[32,14,14]}

            self.batchnorm81_1_8_ = gluon.nn.BatchNorm()
            # batchnorm81_1_8_, output shape: {[32,14,14]}

            self.relu81_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_8_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_8_, output shape: {[32,7,7]}

            self.batchnorm82_1_8_ = gluon.nn.BatchNorm()
            # batchnorm82_1_8_, output shape: {[32,7,7]}

            self.relu82_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_8_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_8_, output shape: {[2048,7,7]}

            self.batchnorm83_1_8_ = gluon.nn.BatchNorm()
            # batchnorm83_1_8_, output shape: {[2048,7,7]}

            self.conv81_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_9_, output shape: {[32,14,14]}

            self.batchnorm81_1_9_ = gluon.nn.BatchNorm()
            # batchnorm81_1_9_, output shape: {[32,14,14]}

            self.relu81_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_9_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_9_, output shape: {[32,7,7]}

            self.batchnorm82_1_9_ = gluon.nn.BatchNorm()
            # batchnorm82_1_9_, output shape: {[32,7,7]}

            self.relu82_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_9_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_9_, output shape: {[2048,7,7]}

            self.batchnorm83_1_9_ = gluon.nn.BatchNorm()
            # batchnorm83_1_9_, output shape: {[2048,7,7]}

            self.conv81_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_10_, output shape: {[32,14,14]}

            self.batchnorm81_1_10_ = gluon.nn.BatchNorm()
            # batchnorm81_1_10_, output shape: {[32,14,14]}

            self.relu81_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_10_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_10_, output shape: {[32,7,7]}

            self.batchnorm82_1_10_ = gluon.nn.BatchNorm()
            # batchnorm82_1_10_, output shape: {[32,7,7]}

            self.relu82_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_10_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_10_, output shape: {[2048,7,7]}

            self.batchnorm83_1_10_ = gluon.nn.BatchNorm()
            # batchnorm83_1_10_, output shape: {[2048,7,7]}

            self.conv81_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_11_, output shape: {[32,14,14]}

            self.batchnorm81_1_11_ = gluon.nn.BatchNorm()
            # batchnorm81_1_11_, output shape: {[32,14,14]}

            self.relu81_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_11_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_11_, output shape: {[32,7,7]}

            self.batchnorm82_1_11_ = gluon.nn.BatchNorm()
            # batchnorm82_1_11_, output shape: {[32,7,7]}

            self.relu82_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_11_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_11_, output shape: {[2048,7,7]}

            self.batchnorm83_1_11_ = gluon.nn.BatchNorm()
            # batchnorm83_1_11_, output shape: {[2048,7,7]}

            self.conv81_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_12_, output shape: {[32,14,14]}

            self.batchnorm81_1_12_ = gluon.nn.BatchNorm()
            # batchnorm81_1_12_, output shape: {[32,14,14]}

            self.relu81_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_12_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_12_, output shape: {[32,7,7]}

            self.batchnorm82_1_12_ = gluon.nn.BatchNorm()
            # batchnorm82_1_12_, output shape: {[32,7,7]}

            self.relu82_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_12_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_12_, output shape: {[2048,7,7]}

            self.batchnorm83_1_12_ = gluon.nn.BatchNorm()
            # batchnorm83_1_12_, output shape: {[2048,7,7]}

            self.conv81_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_13_, output shape: {[32,14,14]}

            self.batchnorm81_1_13_ = gluon.nn.BatchNorm()
            # batchnorm81_1_13_, output shape: {[32,14,14]}

            self.relu81_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_13_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_13_, output shape: {[32,7,7]}

            self.batchnorm82_1_13_ = gluon.nn.BatchNorm()
            # batchnorm82_1_13_, output shape: {[32,7,7]}

            self.relu82_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_13_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_13_, output shape: {[2048,7,7]}

            self.batchnorm83_1_13_ = gluon.nn.BatchNorm()
            # batchnorm83_1_13_, output shape: {[2048,7,7]}

            self.conv81_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_14_, output shape: {[32,14,14]}

            self.batchnorm81_1_14_ = gluon.nn.BatchNorm()
            # batchnorm81_1_14_, output shape: {[32,14,14]}

            self.relu81_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_14_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_14_, output shape: {[32,7,7]}

            self.batchnorm82_1_14_ = gluon.nn.BatchNorm()
            # batchnorm82_1_14_, output shape: {[32,7,7]}

            self.relu82_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_14_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_14_, output shape: {[2048,7,7]}

            self.batchnorm83_1_14_ = gluon.nn.BatchNorm()
            # batchnorm83_1_14_, output shape: {[2048,7,7]}

            self.conv81_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_15_, output shape: {[32,14,14]}

            self.batchnorm81_1_15_ = gluon.nn.BatchNorm()
            # batchnorm81_1_15_, output shape: {[32,14,14]}

            self.relu81_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_15_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_15_, output shape: {[32,7,7]}

            self.batchnorm82_1_15_ = gluon.nn.BatchNorm()
            # batchnorm82_1_15_, output shape: {[32,7,7]}

            self.relu82_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_15_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_15_, output shape: {[2048,7,7]}

            self.batchnorm83_1_15_ = gluon.nn.BatchNorm()
            # batchnorm83_1_15_, output shape: {[2048,7,7]}

            self.conv81_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_16_, output shape: {[32,14,14]}

            self.batchnorm81_1_16_ = gluon.nn.BatchNorm()
            # batchnorm81_1_16_, output shape: {[32,14,14]}

            self.relu81_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_16_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_16_, output shape: {[32,7,7]}

            self.batchnorm82_1_16_ = gluon.nn.BatchNorm()
            # batchnorm82_1_16_, output shape: {[32,7,7]}

            self.relu82_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_16_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_16_, output shape: {[2048,7,7]}

            self.batchnorm83_1_16_ = gluon.nn.BatchNorm()
            # batchnorm83_1_16_, output shape: {[2048,7,7]}

            self.conv81_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_17_, output shape: {[32,14,14]}

            self.batchnorm81_1_17_ = gluon.nn.BatchNorm()
            # batchnorm81_1_17_, output shape: {[32,14,14]}

            self.relu81_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_17_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_17_, output shape: {[32,7,7]}

            self.batchnorm82_1_17_ = gluon.nn.BatchNorm()
            # batchnorm82_1_17_, output shape: {[32,7,7]}

            self.relu82_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_17_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_17_, output shape: {[2048,7,7]}

            self.batchnorm83_1_17_ = gluon.nn.BatchNorm()
            # batchnorm83_1_17_, output shape: {[2048,7,7]}

            self.conv81_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_18_, output shape: {[32,14,14]}

            self.batchnorm81_1_18_ = gluon.nn.BatchNorm()
            # batchnorm81_1_18_, output shape: {[32,14,14]}

            self.relu81_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_18_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_18_, output shape: {[32,7,7]}

            self.batchnorm82_1_18_ = gluon.nn.BatchNorm()
            # batchnorm82_1_18_, output shape: {[32,7,7]}

            self.relu82_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_18_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_18_, output shape: {[2048,7,7]}

            self.batchnorm83_1_18_ = gluon.nn.BatchNorm()
            # batchnorm83_1_18_, output shape: {[2048,7,7]}

            self.conv81_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_19_, output shape: {[32,14,14]}

            self.batchnorm81_1_19_ = gluon.nn.BatchNorm()
            # batchnorm81_1_19_, output shape: {[32,14,14]}

            self.relu81_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_19_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_19_, output shape: {[32,7,7]}

            self.batchnorm82_1_19_ = gluon.nn.BatchNorm()
            # batchnorm82_1_19_, output shape: {[32,7,7]}

            self.relu82_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_19_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_19_, output shape: {[2048,7,7]}

            self.batchnorm83_1_19_ = gluon.nn.BatchNorm()
            # batchnorm83_1_19_, output shape: {[2048,7,7]}

            self.conv81_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_20_, output shape: {[32,14,14]}

            self.batchnorm81_1_20_ = gluon.nn.BatchNorm()
            # batchnorm81_1_20_, output shape: {[32,14,14]}

            self.relu81_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_20_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_20_, output shape: {[32,7,7]}

            self.batchnorm82_1_20_ = gluon.nn.BatchNorm()
            # batchnorm82_1_20_, output shape: {[32,7,7]}

            self.relu82_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_20_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_20_, output shape: {[2048,7,7]}

            self.batchnorm83_1_20_ = gluon.nn.BatchNorm()
            # batchnorm83_1_20_, output shape: {[2048,7,7]}

            self.conv81_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_21_, output shape: {[32,14,14]}

            self.batchnorm81_1_21_ = gluon.nn.BatchNorm()
            # batchnorm81_1_21_, output shape: {[32,14,14]}

            self.relu81_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_21_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_21_, output shape: {[32,7,7]}

            self.batchnorm82_1_21_ = gluon.nn.BatchNorm()
            # batchnorm82_1_21_, output shape: {[32,7,7]}

            self.relu82_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_21_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_21_, output shape: {[2048,7,7]}

            self.batchnorm83_1_21_ = gluon.nn.BatchNorm()
            # batchnorm83_1_21_, output shape: {[2048,7,7]}

            self.conv81_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_22_, output shape: {[32,14,14]}

            self.batchnorm81_1_22_ = gluon.nn.BatchNorm()
            # batchnorm81_1_22_, output shape: {[32,14,14]}

            self.relu81_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_22_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_22_, output shape: {[32,7,7]}

            self.batchnorm82_1_22_ = gluon.nn.BatchNorm()
            # batchnorm82_1_22_, output shape: {[32,7,7]}

            self.relu82_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_22_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_22_, output shape: {[2048,7,7]}

            self.batchnorm83_1_22_ = gluon.nn.BatchNorm()
            # batchnorm83_1_22_, output shape: {[2048,7,7]}

            self.conv81_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_23_, output shape: {[32,14,14]}

            self.batchnorm81_1_23_ = gluon.nn.BatchNorm()
            # batchnorm81_1_23_, output shape: {[32,14,14]}

            self.relu81_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_23_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_23_, output shape: {[32,7,7]}

            self.batchnorm82_1_23_ = gluon.nn.BatchNorm()
            # batchnorm82_1_23_, output shape: {[32,7,7]}

            self.relu82_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_23_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_23_, output shape: {[2048,7,7]}

            self.batchnorm83_1_23_ = gluon.nn.BatchNorm()
            # batchnorm83_1_23_, output shape: {[2048,7,7]}

            self.conv81_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_24_, output shape: {[32,14,14]}

            self.batchnorm81_1_24_ = gluon.nn.BatchNorm()
            # batchnorm81_1_24_, output shape: {[32,14,14]}

            self.relu81_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_24_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_24_, output shape: {[32,7,7]}

            self.batchnorm82_1_24_ = gluon.nn.BatchNorm()
            # batchnorm82_1_24_, output shape: {[32,7,7]}

            self.relu82_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_24_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_24_, output shape: {[2048,7,7]}

            self.batchnorm83_1_24_ = gluon.nn.BatchNorm()
            # batchnorm83_1_24_, output shape: {[2048,7,7]}

            self.conv81_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_25_, output shape: {[32,14,14]}

            self.batchnorm81_1_25_ = gluon.nn.BatchNorm()
            # batchnorm81_1_25_, output shape: {[32,14,14]}

            self.relu81_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_25_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_25_, output shape: {[32,7,7]}

            self.batchnorm82_1_25_ = gluon.nn.BatchNorm()
            # batchnorm82_1_25_, output shape: {[32,7,7]}

            self.relu82_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_25_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_25_, output shape: {[2048,7,7]}

            self.batchnorm83_1_25_ = gluon.nn.BatchNorm()
            # batchnorm83_1_25_, output shape: {[2048,7,7]}

            self.conv81_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_26_, output shape: {[32,14,14]}

            self.batchnorm81_1_26_ = gluon.nn.BatchNorm()
            # batchnorm81_1_26_, output shape: {[32,14,14]}

            self.relu81_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_26_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_26_, output shape: {[32,7,7]}

            self.batchnorm82_1_26_ = gluon.nn.BatchNorm()
            # batchnorm82_1_26_, output shape: {[32,7,7]}

            self.relu82_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_26_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_26_, output shape: {[2048,7,7]}

            self.batchnorm83_1_26_ = gluon.nn.BatchNorm()
            # batchnorm83_1_26_, output shape: {[2048,7,7]}

            self.conv81_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_27_, output shape: {[32,14,14]}

            self.batchnorm81_1_27_ = gluon.nn.BatchNorm()
            # batchnorm81_1_27_, output shape: {[32,14,14]}

            self.relu81_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_27_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_27_, output shape: {[32,7,7]}

            self.batchnorm82_1_27_ = gluon.nn.BatchNorm()
            # batchnorm82_1_27_, output shape: {[32,7,7]}

            self.relu82_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_27_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_27_, output shape: {[2048,7,7]}

            self.batchnorm83_1_27_ = gluon.nn.BatchNorm()
            # batchnorm83_1_27_, output shape: {[2048,7,7]}

            self.conv81_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_28_, output shape: {[32,14,14]}

            self.batchnorm81_1_28_ = gluon.nn.BatchNorm()
            # batchnorm81_1_28_, output shape: {[32,14,14]}

            self.relu81_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_28_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_28_, output shape: {[32,7,7]}

            self.batchnorm82_1_28_ = gluon.nn.BatchNorm()
            # batchnorm82_1_28_, output shape: {[32,7,7]}

            self.relu82_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_28_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_28_, output shape: {[2048,7,7]}

            self.batchnorm83_1_28_ = gluon.nn.BatchNorm()
            # batchnorm83_1_28_, output shape: {[2048,7,7]}

            self.conv81_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_29_, output shape: {[32,14,14]}

            self.batchnorm81_1_29_ = gluon.nn.BatchNorm()
            # batchnorm81_1_29_, output shape: {[32,14,14]}

            self.relu81_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_29_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_29_, output shape: {[32,7,7]}

            self.batchnorm82_1_29_ = gluon.nn.BatchNorm()
            # batchnorm82_1_29_, output shape: {[32,7,7]}

            self.relu82_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_29_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_29_, output shape: {[2048,7,7]}

            self.batchnorm83_1_29_ = gluon.nn.BatchNorm()
            # batchnorm83_1_29_, output shape: {[2048,7,7]}

            self.conv81_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_30_, output shape: {[32,14,14]}

            self.batchnorm81_1_30_ = gluon.nn.BatchNorm()
            # batchnorm81_1_30_, output shape: {[32,14,14]}

            self.relu81_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_30_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_30_, output shape: {[32,7,7]}

            self.batchnorm82_1_30_ = gluon.nn.BatchNorm()
            # batchnorm82_1_30_, output shape: {[32,7,7]}

            self.relu82_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_30_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_30_, output shape: {[2048,7,7]}

            self.batchnorm83_1_30_ = gluon.nn.BatchNorm()
            # batchnorm83_1_30_, output shape: {[2048,7,7]}

            self.conv81_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_31_, output shape: {[32,14,14]}

            self.batchnorm81_1_31_ = gluon.nn.BatchNorm()
            # batchnorm81_1_31_, output shape: {[32,14,14]}

            self.relu81_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_31_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_31_, output shape: {[32,7,7]}

            self.batchnorm82_1_31_ = gluon.nn.BatchNorm()
            # batchnorm82_1_31_, output shape: {[32,7,7]}

            self.relu82_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_31_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_31_, output shape: {[2048,7,7]}

            self.batchnorm83_1_31_ = gluon.nn.BatchNorm()
            # batchnorm83_1_31_, output shape: {[2048,7,7]}

            self.conv81_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv81_1_32_, output shape: {[32,14,14]}

            self.batchnorm81_1_32_ = gluon.nn.BatchNorm()
            # batchnorm81_1_32_, output shape: {[32,14,14]}

            self.relu81_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv82_1_32_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv82_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv82_1_32_, output shape: {[32,7,7]}

            self.batchnorm82_1_32_ = gluon.nn.BatchNorm()
            # batchnorm82_1_32_, output shape: {[32,7,7]}

            self.relu82_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv83_1_32_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv83_1_32_, output shape: {[2048,7,7]}

            self.batchnorm83_1_32_ = gluon.nn.BatchNorm()
            # batchnorm83_1_32_, output shape: {[2048,7,7]}

            self.conv80_2_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv80_2_, output shape: {[2048,7,7]}

            self.batchnorm80_2_ = gluon.nn.BatchNorm()
            # batchnorm80_2_, output shape: {[2048,7,7]}

            self.relu85_ = gluon.nn.Activation(activation='relu')
            self.conv87_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_1_, output shape: {[32,7,7]}

            self.batchnorm87_1_1_ = gluon.nn.BatchNorm()
            # batchnorm87_1_1_, output shape: {[32,7,7]}

            self.relu87_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_1_, output shape: {[32,7,7]}

            self.batchnorm88_1_1_ = gluon.nn.BatchNorm()
            # batchnorm88_1_1_, output shape: {[32,7,7]}

            self.relu88_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_1_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_1_, output shape: {[2048,7,7]}

            self.batchnorm89_1_1_ = gluon.nn.BatchNorm()
            # batchnorm89_1_1_, output shape: {[2048,7,7]}

            self.conv87_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_2_, output shape: {[32,7,7]}

            self.batchnorm87_1_2_ = gluon.nn.BatchNorm()
            # batchnorm87_1_2_, output shape: {[32,7,7]}

            self.relu87_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_2_, output shape: {[32,7,7]}

            self.batchnorm88_1_2_ = gluon.nn.BatchNorm()
            # batchnorm88_1_2_, output shape: {[32,7,7]}

            self.relu88_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_2_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_2_, output shape: {[2048,7,7]}

            self.batchnorm89_1_2_ = gluon.nn.BatchNorm()
            # batchnorm89_1_2_, output shape: {[2048,7,7]}

            self.conv87_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_3_, output shape: {[32,7,7]}

            self.batchnorm87_1_3_ = gluon.nn.BatchNorm()
            # batchnorm87_1_3_, output shape: {[32,7,7]}

            self.relu87_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_3_, output shape: {[32,7,7]}

            self.batchnorm88_1_3_ = gluon.nn.BatchNorm()
            # batchnorm88_1_3_, output shape: {[32,7,7]}

            self.relu88_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_3_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_3_, output shape: {[2048,7,7]}

            self.batchnorm89_1_3_ = gluon.nn.BatchNorm()
            # batchnorm89_1_3_, output shape: {[2048,7,7]}

            self.conv87_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_4_, output shape: {[32,7,7]}

            self.batchnorm87_1_4_ = gluon.nn.BatchNorm()
            # batchnorm87_1_4_, output shape: {[32,7,7]}

            self.relu87_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_4_, output shape: {[32,7,7]}

            self.batchnorm88_1_4_ = gluon.nn.BatchNorm()
            # batchnorm88_1_4_, output shape: {[32,7,7]}

            self.relu88_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_4_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_4_, output shape: {[2048,7,7]}

            self.batchnorm89_1_4_ = gluon.nn.BatchNorm()
            # batchnorm89_1_4_, output shape: {[2048,7,7]}

            self.conv87_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_5_, output shape: {[32,7,7]}

            self.batchnorm87_1_5_ = gluon.nn.BatchNorm()
            # batchnorm87_1_5_, output shape: {[32,7,7]}

            self.relu87_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_5_, output shape: {[32,7,7]}

            self.batchnorm88_1_5_ = gluon.nn.BatchNorm()
            # batchnorm88_1_5_, output shape: {[32,7,7]}

            self.relu88_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_5_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_5_, output shape: {[2048,7,7]}

            self.batchnorm89_1_5_ = gluon.nn.BatchNorm()
            # batchnorm89_1_5_, output shape: {[2048,7,7]}

            self.conv87_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_6_, output shape: {[32,7,7]}

            self.batchnorm87_1_6_ = gluon.nn.BatchNorm()
            # batchnorm87_1_6_, output shape: {[32,7,7]}

            self.relu87_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_6_, output shape: {[32,7,7]}

            self.batchnorm88_1_6_ = gluon.nn.BatchNorm()
            # batchnorm88_1_6_, output shape: {[32,7,7]}

            self.relu88_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_6_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_6_, output shape: {[2048,7,7]}

            self.batchnorm89_1_6_ = gluon.nn.BatchNorm()
            # batchnorm89_1_6_, output shape: {[2048,7,7]}

            self.conv87_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_7_, output shape: {[32,7,7]}

            self.batchnorm87_1_7_ = gluon.nn.BatchNorm()
            # batchnorm87_1_7_, output shape: {[32,7,7]}

            self.relu87_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_7_, output shape: {[32,7,7]}

            self.batchnorm88_1_7_ = gluon.nn.BatchNorm()
            # batchnorm88_1_7_, output shape: {[32,7,7]}

            self.relu88_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_7_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_7_, output shape: {[2048,7,7]}

            self.batchnorm89_1_7_ = gluon.nn.BatchNorm()
            # batchnorm89_1_7_, output shape: {[2048,7,7]}

            self.conv87_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_8_, output shape: {[32,7,7]}

            self.batchnorm87_1_8_ = gluon.nn.BatchNorm()
            # batchnorm87_1_8_, output shape: {[32,7,7]}

            self.relu87_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_8_, output shape: {[32,7,7]}

            self.batchnorm88_1_8_ = gluon.nn.BatchNorm()
            # batchnorm88_1_8_, output shape: {[32,7,7]}

            self.relu88_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_8_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_8_, output shape: {[2048,7,7]}

            self.batchnorm89_1_8_ = gluon.nn.BatchNorm()
            # batchnorm89_1_8_, output shape: {[2048,7,7]}

            self.conv87_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_9_, output shape: {[32,7,7]}

            self.batchnorm87_1_9_ = gluon.nn.BatchNorm()
            # batchnorm87_1_9_, output shape: {[32,7,7]}

            self.relu87_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_9_, output shape: {[32,7,7]}

            self.batchnorm88_1_9_ = gluon.nn.BatchNorm()
            # batchnorm88_1_9_, output shape: {[32,7,7]}

            self.relu88_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_9_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_9_, output shape: {[2048,7,7]}

            self.batchnorm89_1_9_ = gluon.nn.BatchNorm()
            # batchnorm89_1_9_, output shape: {[2048,7,7]}

            self.conv87_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_10_, output shape: {[32,7,7]}

            self.batchnorm87_1_10_ = gluon.nn.BatchNorm()
            # batchnorm87_1_10_, output shape: {[32,7,7]}

            self.relu87_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_10_, output shape: {[32,7,7]}

            self.batchnorm88_1_10_ = gluon.nn.BatchNorm()
            # batchnorm88_1_10_, output shape: {[32,7,7]}

            self.relu88_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_10_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_10_, output shape: {[2048,7,7]}

            self.batchnorm89_1_10_ = gluon.nn.BatchNorm()
            # batchnorm89_1_10_, output shape: {[2048,7,7]}

            self.conv87_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_11_, output shape: {[32,7,7]}

            self.batchnorm87_1_11_ = gluon.nn.BatchNorm()
            # batchnorm87_1_11_, output shape: {[32,7,7]}

            self.relu87_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_11_, output shape: {[32,7,7]}

            self.batchnorm88_1_11_ = gluon.nn.BatchNorm()
            # batchnorm88_1_11_, output shape: {[32,7,7]}

            self.relu88_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_11_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_11_, output shape: {[2048,7,7]}

            self.batchnorm89_1_11_ = gluon.nn.BatchNorm()
            # batchnorm89_1_11_, output shape: {[2048,7,7]}

            self.conv87_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_12_, output shape: {[32,7,7]}

            self.batchnorm87_1_12_ = gluon.nn.BatchNorm()
            # batchnorm87_1_12_, output shape: {[32,7,7]}

            self.relu87_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_12_, output shape: {[32,7,7]}

            self.batchnorm88_1_12_ = gluon.nn.BatchNorm()
            # batchnorm88_1_12_, output shape: {[32,7,7]}

            self.relu88_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_12_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_12_, output shape: {[2048,7,7]}

            self.batchnorm89_1_12_ = gluon.nn.BatchNorm()
            # batchnorm89_1_12_, output shape: {[2048,7,7]}

            self.conv87_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_13_, output shape: {[32,7,7]}

            self.batchnorm87_1_13_ = gluon.nn.BatchNorm()
            # batchnorm87_1_13_, output shape: {[32,7,7]}

            self.relu87_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_13_, output shape: {[32,7,7]}

            self.batchnorm88_1_13_ = gluon.nn.BatchNorm()
            # batchnorm88_1_13_, output shape: {[32,7,7]}

            self.relu88_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_13_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_13_, output shape: {[2048,7,7]}

            self.batchnorm89_1_13_ = gluon.nn.BatchNorm()
            # batchnorm89_1_13_, output shape: {[2048,7,7]}

            self.conv87_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_14_, output shape: {[32,7,7]}

            self.batchnorm87_1_14_ = gluon.nn.BatchNorm()
            # batchnorm87_1_14_, output shape: {[32,7,7]}

            self.relu87_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_14_, output shape: {[32,7,7]}

            self.batchnorm88_1_14_ = gluon.nn.BatchNorm()
            # batchnorm88_1_14_, output shape: {[32,7,7]}

            self.relu88_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_14_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_14_, output shape: {[2048,7,7]}

            self.batchnorm89_1_14_ = gluon.nn.BatchNorm()
            # batchnorm89_1_14_, output shape: {[2048,7,7]}

            self.conv87_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_15_, output shape: {[32,7,7]}

            self.batchnorm87_1_15_ = gluon.nn.BatchNorm()
            # batchnorm87_1_15_, output shape: {[32,7,7]}

            self.relu87_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_15_, output shape: {[32,7,7]}

            self.batchnorm88_1_15_ = gluon.nn.BatchNorm()
            # batchnorm88_1_15_, output shape: {[32,7,7]}

            self.relu88_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_15_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_15_, output shape: {[2048,7,7]}

            self.batchnorm89_1_15_ = gluon.nn.BatchNorm()
            # batchnorm89_1_15_, output shape: {[2048,7,7]}

            self.conv87_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_16_, output shape: {[32,7,7]}

            self.batchnorm87_1_16_ = gluon.nn.BatchNorm()
            # batchnorm87_1_16_, output shape: {[32,7,7]}

            self.relu87_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_16_, output shape: {[32,7,7]}

            self.batchnorm88_1_16_ = gluon.nn.BatchNorm()
            # batchnorm88_1_16_, output shape: {[32,7,7]}

            self.relu88_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_16_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_16_, output shape: {[2048,7,7]}

            self.batchnorm89_1_16_ = gluon.nn.BatchNorm()
            # batchnorm89_1_16_, output shape: {[2048,7,7]}

            self.conv87_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_17_, output shape: {[32,7,7]}

            self.batchnorm87_1_17_ = gluon.nn.BatchNorm()
            # batchnorm87_1_17_, output shape: {[32,7,7]}

            self.relu87_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_17_, output shape: {[32,7,7]}

            self.batchnorm88_1_17_ = gluon.nn.BatchNorm()
            # batchnorm88_1_17_, output shape: {[32,7,7]}

            self.relu88_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_17_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_17_, output shape: {[2048,7,7]}

            self.batchnorm89_1_17_ = gluon.nn.BatchNorm()
            # batchnorm89_1_17_, output shape: {[2048,7,7]}

            self.conv87_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_18_, output shape: {[32,7,7]}

            self.batchnorm87_1_18_ = gluon.nn.BatchNorm()
            # batchnorm87_1_18_, output shape: {[32,7,7]}

            self.relu87_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_18_, output shape: {[32,7,7]}

            self.batchnorm88_1_18_ = gluon.nn.BatchNorm()
            # batchnorm88_1_18_, output shape: {[32,7,7]}

            self.relu88_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_18_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_18_, output shape: {[2048,7,7]}

            self.batchnorm89_1_18_ = gluon.nn.BatchNorm()
            # batchnorm89_1_18_, output shape: {[2048,7,7]}

            self.conv87_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_19_, output shape: {[32,7,7]}

            self.batchnorm87_1_19_ = gluon.nn.BatchNorm()
            # batchnorm87_1_19_, output shape: {[32,7,7]}

            self.relu87_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_19_, output shape: {[32,7,7]}

            self.batchnorm88_1_19_ = gluon.nn.BatchNorm()
            # batchnorm88_1_19_, output shape: {[32,7,7]}

            self.relu88_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_19_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_19_, output shape: {[2048,7,7]}

            self.batchnorm89_1_19_ = gluon.nn.BatchNorm()
            # batchnorm89_1_19_, output shape: {[2048,7,7]}

            self.conv87_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_20_, output shape: {[32,7,7]}

            self.batchnorm87_1_20_ = gluon.nn.BatchNorm()
            # batchnorm87_1_20_, output shape: {[32,7,7]}

            self.relu87_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_20_, output shape: {[32,7,7]}

            self.batchnorm88_1_20_ = gluon.nn.BatchNorm()
            # batchnorm88_1_20_, output shape: {[32,7,7]}

            self.relu88_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_20_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_20_, output shape: {[2048,7,7]}

            self.batchnorm89_1_20_ = gluon.nn.BatchNorm()
            # batchnorm89_1_20_, output shape: {[2048,7,7]}

            self.conv87_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_21_, output shape: {[32,7,7]}

            self.batchnorm87_1_21_ = gluon.nn.BatchNorm()
            # batchnorm87_1_21_, output shape: {[32,7,7]}

            self.relu87_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_21_, output shape: {[32,7,7]}

            self.batchnorm88_1_21_ = gluon.nn.BatchNorm()
            # batchnorm88_1_21_, output shape: {[32,7,7]}

            self.relu88_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_21_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_21_, output shape: {[2048,7,7]}

            self.batchnorm89_1_21_ = gluon.nn.BatchNorm()
            # batchnorm89_1_21_, output shape: {[2048,7,7]}

            self.conv87_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_22_, output shape: {[32,7,7]}

            self.batchnorm87_1_22_ = gluon.nn.BatchNorm()
            # batchnorm87_1_22_, output shape: {[32,7,7]}

            self.relu87_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_22_, output shape: {[32,7,7]}

            self.batchnorm88_1_22_ = gluon.nn.BatchNorm()
            # batchnorm88_1_22_, output shape: {[32,7,7]}

            self.relu88_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_22_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_22_, output shape: {[2048,7,7]}

            self.batchnorm89_1_22_ = gluon.nn.BatchNorm()
            # batchnorm89_1_22_, output shape: {[2048,7,7]}

            self.conv87_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_23_, output shape: {[32,7,7]}

            self.batchnorm87_1_23_ = gluon.nn.BatchNorm()
            # batchnorm87_1_23_, output shape: {[32,7,7]}

            self.relu87_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_23_, output shape: {[32,7,7]}

            self.batchnorm88_1_23_ = gluon.nn.BatchNorm()
            # batchnorm88_1_23_, output shape: {[32,7,7]}

            self.relu88_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_23_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_23_, output shape: {[2048,7,7]}

            self.batchnorm89_1_23_ = gluon.nn.BatchNorm()
            # batchnorm89_1_23_, output shape: {[2048,7,7]}

            self.conv87_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_24_, output shape: {[32,7,7]}

            self.batchnorm87_1_24_ = gluon.nn.BatchNorm()
            # batchnorm87_1_24_, output shape: {[32,7,7]}

            self.relu87_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_24_, output shape: {[32,7,7]}

            self.batchnorm88_1_24_ = gluon.nn.BatchNorm()
            # batchnorm88_1_24_, output shape: {[32,7,7]}

            self.relu88_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_24_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_24_, output shape: {[2048,7,7]}

            self.batchnorm89_1_24_ = gluon.nn.BatchNorm()
            # batchnorm89_1_24_, output shape: {[2048,7,7]}

            self.conv87_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_25_, output shape: {[32,7,7]}

            self.batchnorm87_1_25_ = gluon.nn.BatchNorm()
            # batchnorm87_1_25_, output shape: {[32,7,7]}

            self.relu87_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_25_, output shape: {[32,7,7]}

            self.batchnorm88_1_25_ = gluon.nn.BatchNorm()
            # batchnorm88_1_25_, output shape: {[32,7,7]}

            self.relu88_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_25_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_25_, output shape: {[2048,7,7]}

            self.batchnorm89_1_25_ = gluon.nn.BatchNorm()
            # batchnorm89_1_25_, output shape: {[2048,7,7]}

            self.conv87_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_26_, output shape: {[32,7,7]}

            self.batchnorm87_1_26_ = gluon.nn.BatchNorm()
            # batchnorm87_1_26_, output shape: {[32,7,7]}

            self.relu87_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_26_, output shape: {[32,7,7]}

            self.batchnorm88_1_26_ = gluon.nn.BatchNorm()
            # batchnorm88_1_26_, output shape: {[32,7,7]}

            self.relu88_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_26_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_26_, output shape: {[2048,7,7]}

            self.batchnorm89_1_26_ = gluon.nn.BatchNorm()
            # batchnorm89_1_26_, output shape: {[2048,7,7]}

            self.conv87_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_27_, output shape: {[32,7,7]}

            self.batchnorm87_1_27_ = gluon.nn.BatchNorm()
            # batchnorm87_1_27_, output shape: {[32,7,7]}

            self.relu87_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_27_, output shape: {[32,7,7]}

            self.batchnorm88_1_27_ = gluon.nn.BatchNorm()
            # batchnorm88_1_27_, output shape: {[32,7,7]}

            self.relu88_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_27_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_27_, output shape: {[2048,7,7]}

            self.batchnorm89_1_27_ = gluon.nn.BatchNorm()
            # batchnorm89_1_27_, output shape: {[2048,7,7]}

            self.conv87_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_28_, output shape: {[32,7,7]}

            self.batchnorm87_1_28_ = gluon.nn.BatchNorm()
            # batchnorm87_1_28_, output shape: {[32,7,7]}

            self.relu87_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_28_, output shape: {[32,7,7]}

            self.batchnorm88_1_28_ = gluon.nn.BatchNorm()
            # batchnorm88_1_28_, output shape: {[32,7,7]}

            self.relu88_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_28_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_28_, output shape: {[2048,7,7]}

            self.batchnorm89_1_28_ = gluon.nn.BatchNorm()
            # batchnorm89_1_28_, output shape: {[2048,7,7]}

            self.conv87_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_29_, output shape: {[32,7,7]}

            self.batchnorm87_1_29_ = gluon.nn.BatchNorm()
            # batchnorm87_1_29_, output shape: {[32,7,7]}

            self.relu87_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_29_, output shape: {[32,7,7]}

            self.batchnorm88_1_29_ = gluon.nn.BatchNorm()
            # batchnorm88_1_29_, output shape: {[32,7,7]}

            self.relu88_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_29_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_29_, output shape: {[2048,7,7]}

            self.batchnorm89_1_29_ = gluon.nn.BatchNorm()
            # batchnorm89_1_29_, output shape: {[2048,7,7]}

            self.conv87_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_30_, output shape: {[32,7,7]}

            self.batchnorm87_1_30_ = gluon.nn.BatchNorm()
            # batchnorm87_1_30_, output shape: {[32,7,7]}

            self.relu87_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_30_, output shape: {[32,7,7]}

            self.batchnorm88_1_30_ = gluon.nn.BatchNorm()
            # batchnorm88_1_30_, output shape: {[32,7,7]}

            self.relu88_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_30_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_30_, output shape: {[2048,7,7]}

            self.batchnorm89_1_30_ = gluon.nn.BatchNorm()
            # batchnorm89_1_30_, output shape: {[2048,7,7]}

            self.conv87_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_31_, output shape: {[32,7,7]}

            self.batchnorm87_1_31_ = gluon.nn.BatchNorm()
            # batchnorm87_1_31_, output shape: {[32,7,7]}

            self.relu87_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_31_, output shape: {[32,7,7]}

            self.batchnorm88_1_31_ = gluon.nn.BatchNorm()
            # batchnorm88_1_31_, output shape: {[32,7,7]}

            self.relu88_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_31_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_31_, output shape: {[2048,7,7]}

            self.batchnorm89_1_31_ = gluon.nn.BatchNorm()
            # batchnorm89_1_31_, output shape: {[2048,7,7]}

            self.conv87_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv87_1_32_, output shape: {[32,7,7]}

            self.batchnorm87_1_32_ = gluon.nn.BatchNorm()
            # batchnorm87_1_32_, output shape: {[32,7,7]}

            self.relu87_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv88_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv88_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv88_1_32_, output shape: {[32,7,7]}

            self.batchnorm88_1_32_ = gluon.nn.BatchNorm()
            # batchnorm88_1_32_, output shape: {[32,7,7]}

            self.relu88_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv89_1_32_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv89_1_32_, output shape: {[2048,7,7]}

            self.batchnorm89_1_32_ = gluon.nn.BatchNorm()
            # batchnorm89_1_32_, output shape: {[2048,7,7]}

            self.relu91_ = gluon.nn.Activation(activation='relu')
            self.conv93_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_1_, output shape: {[32,7,7]}

            self.batchnorm93_1_1_ = gluon.nn.BatchNorm()
            # batchnorm93_1_1_, output shape: {[32,7,7]}

            self.relu93_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_1_, output shape: {[32,7,7]}

            self.batchnorm94_1_1_ = gluon.nn.BatchNorm()
            # batchnorm94_1_1_, output shape: {[32,7,7]}

            self.relu94_1_1_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_1_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_1_, output shape: {[2048,7,7]}

            self.batchnorm95_1_1_ = gluon.nn.BatchNorm()
            # batchnorm95_1_1_, output shape: {[2048,7,7]}

            self.conv93_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_2_, output shape: {[32,7,7]}

            self.batchnorm93_1_2_ = gluon.nn.BatchNorm()
            # batchnorm93_1_2_, output shape: {[32,7,7]}

            self.relu93_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_2_, output shape: {[32,7,7]}

            self.batchnorm94_1_2_ = gluon.nn.BatchNorm()
            # batchnorm94_1_2_, output shape: {[32,7,7]}

            self.relu94_1_2_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_2_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_2_, output shape: {[2048,7,7]}

            self.batchnorm95_1_2_ = gluon.nn.BatchNorm()
            # batchnorm95_1_2_, output shape: {[2048,7,7]}

            self.conv93_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_3_, output shape: {[32,7,7]}

            self.batchnorm93_1_3_ = gluon.nn.BatchNorm()
            # batchnorm93_1_3_, output shape: {[32,7,7]}

            self.relu93_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_3_, output shape: {[32,7,7]}

            self.batchnorm94_1_3_ = gluon.nn.BatchNorm()
            # batchnorm94_1_3_, output shape: {[32,7,7]}

            self.relu94_1_3_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_3_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_3_, output shape: {[2048,7,7]}

            self.batchnorm95_1_3_ = gluon.nn.BatchNorm()
            # batchnorm95_1_3_, output shape: {[2048,7,7]}

            self.conv93_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_4_, output shape: {[32,7,7]}

            self.batchnorm93_1_4_ = gluon.nn.BatchNorm()
            # batchnorm93_1_4_, output shape: {[32,7,7]}

            self.relu93_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_4_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_4_, output shape: {[32,7,7]}

            self.batchnorm94_1_4_ = gluon.nn.BatchNorm()
            # batchnorm94_1_4_, output shape: {[32,7,7]}

            self.relu94_1_4_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_4_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_4_, output shape: {[2048,7,7]}

            self.batchnorm95_1_4_ = gluon.nn.BatchNorm()
            # batchnorm95_1_4_, output shape: {[2048,7,7]}

            self.conv93_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_5_, output shape: {[32,7,7]}

            self.batchnorm93_1_5_ = gluon.nn.BatchNorm()
            # batchnorm93_1_5_, output shape: {[32,7,7]}

            self.relu93_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_5_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_5_, output shape: {[32,7,7]}

            self.batchnorm94_1_5_ = gluon.nn.BatchNorm()
            # batchnorm94_1_5_, output shape: {[32,7,7]}

            self.relu94_1_5_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_5_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_5_, output shape: {[2048,7,7]}

            self.batchnorm95_1_5_ = gluon.nn.BatchNorm()
            # batchnorm95_1_5_, output shape: {[2048,7,7]}

            self.conv93_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_6_, output shape: {[32,7,7]}

            self.batchnorm93_1_6_ = gluon.nn.BatchNorm()
            # batchnorm93_1_6_, output shape: {[32,7,7]}

            self.relu93_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_6_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_6_, output shape: {[32,7,7]}

            self.batchnorm94_1_6_ = gluon.nn.BatchNorm()
            # batchnorm94_1_6_, output shape: {[32,7,7]}

            self.relu94_1_6_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_6_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_6_, output shape: {[2048,7,7]}

            self.batchnorm95_1_6_ = gluon.nn.BatchNorm()
            # batchnorm95_1_6_, output shape: {[2048,7,7]}

            self.conv93_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_7_, output shape: {[32,7,7]}

            self.batchnorm93_1_7_ = gluon.nn.BatchNorm()
            # batchnorm93_1_7_, output shape: {[32,7,7]}

            self.relu93_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_7_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_7_, output shape: {[32,7,7]}

            self.batchnorm94_1_7_ = gluon.nn.BatchNorm()
            # batchnorm94_1_7_, output shape: {[32,7,7]}

            self.relu94_1_7_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_7_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_7_, output shape: {[2048,7,7]}

            self.batchnorm95_1_7_ = gluon.nn.BatchNorm()
            # batchnorm95_1_7_, output shape: {[2048,7,7]}

            self.conv93_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_8_, output shape: {[32,7,7]}

            self.batchnorm93_1_8_ = gluon.nn.BatchNorm()
            # batchnorm93_1_8_, output shape: {[32,7,7]}

            self.relu93_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_8_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_8_, output shape: {[32,7,7]}

            self.batchnorm94_1_8_ = gluon.nn.BatchNorm()
            # batchnorm94_1_8_, output shape: {[32,7,7]}

            self.relu94_1_8_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_8_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_8_, output shape: {[2048,7,7]}

            self.batchnorm95_1_8_ = gluon.nn.BatchNorm()
            # batchnorm95_1_8_, output shape: {[2048,7,7]}

            self.conv93_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_9_, output shape: {[32,7,7]}

            self.batchnorm93_1_9_ = gluon.nn.BatchNorm()
            # batchnorm93_1_9_, output shape: {[32,7,7]}

            self.relu93_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_9_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_9_, output shape: {[32,7,7]}

            self.batchnorm94_1_9_ = gluon.nn.BatchNorm()
            # batchnorm94_1_9_, output shape: {[32,7,7]}

            self.relu94_1_9_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_9_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_9_, output shape: {[2048,7,7]}

            self.batchnorm95_1_9_ = gluon.nn.BatchNorm()
            # batchnorm95_1_9_, output shape: {[2048,7,7]}

            self.conv93_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_10_, output shape: {[32,7,7]}

            self.batchnorm93_1_10_ = gluon.nn.BatchNorm()
            # batchnorm93_1_10_, output shape: {[32,7,7]}

            self.relu93_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_10_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_10_, output shape: {[32,7,7]}

            self.batchnorm94_1_10_ = gluon.nn.BatchNorm()
            # batchnorm94_1_10_, output shape: {[32,7,7]}

            self.relu94_1_10_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_10_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_10_, output shape: {[2048,7,7]}

            self.batchnorm95_1_10_ = gluon.nn.BatchNorm()
            # batchnorm95_1_10_, output shape: {[2048,7,7]}

            self.conv93_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_11_, output shape: {[32,7,7]}

            self.batchnorm93_1_11_ = gluon.nn.BatchNorm()
            # batchnorm93_1_11_, output shape: {[32,7,7]}

            self.relu93_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_11_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_11_, output shape: {[32,7,7]}

            self.batchnorm94_1_11_ = gluon.nn.BatchNorm()
            # batchnorm94_1_11_, output shape: {[32,7,7]}

            self.relu94_1_11_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_11_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_11_, output shape: {[2048,7,7]}

            self.batchnorm95_1_11_ = gluon.nn.BatchNorm()
            # batchnorm95_1_11_, output shape: {[2048,7,7]}

            self.conv93_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_12_, output shape: {[32,7,7]}

            self.batchnorm93_1_12_ = gluon.nn.BatchNorm()
            # batchnorm93_1_12_, output shape: {[32,7,7]}

            self.relu93_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_12_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_12_, output shape: {[32,7,7]}

            self.batchnorm94_1_12_ = gluon.nn.BatchNorm()
            # batchnorm94_1_12_, output shape: {[32,7,7]}

            self.relu94_1_12_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_12_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_12_, output shape: {[2048,7,7]}

            self.batchnorm95_1_12_ = gluon.nn.BatchNorm()
            # batchnorm95_1_12_, output shape: {[2048,7,7]}

            self.conv93_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_13_, output shape: {[32,7,7]}

            self.batchnorm93_1_13_ = gluon.nn.BatchNorm()
            # batchnorm93_1_13_, output shape: {[32,7,7]}

            self.relu93_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_13_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_13_, output shape: {[32,7,7]}

            self.batchnorm94_1_13_ = gluon.nn.BatchNorm()
            # batchnorm94_1_13_, output shape: {[32,7,7]}

            self.relu94_1_13_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_13_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_13_, output shape: {[2048,7,7]}

            self.batchnorm95_1_13_ = gluon.nn.BatchNorm()
            # batchnorm95_1_13_, output shape: {[2048,7,7]}

            self.conv93_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_14_, output shape: {[32,7,7]}

            self.batchnorm93_1_14_ = gluon.nn.BatchNorm()
            # batchnorm93_1_14_, output shape: {[32,7,7]}

            self.relu93_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_14_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_14_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_14_, output shape: {[32,7,7]}

            self.batchnorm94_1_14_ = gluon.nn.BatchNorm()
            # batchnorm94_1_14_, output shape: {[32,7,7]}

            self.relu94_1_14_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_14_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_14_, output shape: {[2048,7,7]}

            self.batchnorm95_1_14_ = gluon.nn.BatchNorm()
            # batchnorm95_1_14_, output shape: {[2048,7,7]}

            self.conv93_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_15_, output shape: {[32,7,7]}

            self.batchnorm93_1_15_ = gluon.nn.BatchNorm()
            # batchnorm93_1_15_, output shape: {[32,7,7]}

            self.relu93_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_15_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_15_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_15_, output shape: {[32,7,7]}

            self.batchnorm94_1_15_ = gluon.nn.BatchNorm()
            # batchnorm94_1_15_, output shape: {[32,7,7]}

            self.relu94_1_15_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_15_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_15_, output shape: {[2048,7,7]}

            self.batchnorm95_1_15_ = gluon.nn.BatchNorm()
            # batchnorm95_1_15_, output shape: {[2048,7,7]}

            self.conv93_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_16_, output shape: {[32,7,7]}

            self.batchnorm93_1_16_ = gluon.nn.BatchNorm()
            # batchnorm93_1_16_, output shape: {[32,7,7]}

            self.relu93_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_16_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_16_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_16_, output shape: {[32,7,7]}

            self.batchnorm94_1_16_ = gluon.nn.BatchNorm()
            # batchnorm94_1_16_, output shape: {[32,7,7]}

            self.relu94_1_16_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_16_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_16_, output shape: {[2048,7,7]}

            self.batchnorm95_1_16_ = gluon.nn.BatchNorm()
            # batchnorm95_1_16_, output shape: {[2048,7,7]}

            self.conv93_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_17_, output shape: {[32,7,7]}

            self.batchnorm93_1_17_ = gluon.nn.BatchNorm()
            # batchnorm93_1_17_, output shape: {[32,7,7]}

            self.relu93_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_17_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_17_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_17_, output shape: {[32,7,7]}

            self.batchnorm94_1_17_ = gluon.nn.BatchNorm()
            # batchnorm94_1_17_, output shape: {[32,7,7]}

            self.relu94_1_17_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_17_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_17_, output shape: {[2048,7,7]}

            self.batchnorm95_1_17_ = gluon.nn.BatchNorm()
            # batchnorm95_1_17_, output shape: {[2048,7,7]}

            self.conv93_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_18_, output shape: {[32,7,7]}

            self.batchnorm93_1_18_ = gluon.nn.BatchNorm()
            # batchnorm93_1_18_, output shape: {[32,7,7]}

            self.relu93_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_18_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_18_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_18_, output shape: {[32,7,7]}

            self.batchnorm94_1_18_ = gluon.nn.BatchNorm()
            # batchnorm94_1_18_, output shape: {[32,7,7]}

            self.relu94_1_18_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_18_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_18_, output shape: {[2048,7,7]}

            self.batchnorm95_1_18_ = gluon.nn.BatchNorm()
            # batchnorm95_1_18_, output shape: {[2048,7,7]}

            self.conv93_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_19_, output shape: {[32,7,7]}

            self.batchnorm93_1_19_ = gluon.nn.BatchNorm()
            # batchnorm93_1_19_, output shape: {[32,7,7]}

            self.relu93_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_19_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_19_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_19_, output shape: {[32,7,7]}

            self.batchnorm94_1_19_ = gluon.nn.BatchNorm()
            # batchnorm94_1_19_, output shape: {[32,7,7]}

            self.relu94_1_19_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_19_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_19_, output shape: {[2048,7,7]}

            self.batchnorm95_1_19_ = gluon.nn.BatchNorm()
            # batchnorm95_1_19_, output shape: {[2048,7,7]}

            self.conv93_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_20_, output shape: {[32,7,7]}

            self.batchnorm93_1_20_ = gluon.nn.BatchNorm()
            # batchnorm93_1_20_, output shape: {[32,7,7]}

            self.relu93_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_20_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_20_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_20_, output shape: {[32,7,7]}

            self.batchnorm94_1_20_ = gluon.nn.BatchNorm()
            # batchnorm94_1_20_, output shape: {[32,7,7]}

            self.relu94_1_20_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_20_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_20_, output shape: {[2048,7,7]}

            self.batchnorm95_1_20_ = gluon.nn.BatchNorm()
            # batchnorm95_1_20_, output shape: {[2048,7,7]}

            self.conv93_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_21_, output shape: {[32,7,7]}

            self.batchnorm93_1_21_ = gluon.nn.BatchNorm()
            # batchnorm93_1_21_, output shape: {[32,7,7]}

            self.relu93_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_21_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_21_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_21_, output shape: {[32,7,7]}

            self.batchnorm94_1_21_ = gluon.nn.BatchNorm()
            # batchnorm94_1_21_, output shape: {[32,7,7]}

            self.relu94_1_21_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_21_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_21_, output shape: {[2048,7,7]}

            self.batchnorm95_1_21_ = gluon.nn.BatchNorm()
            # batchnorm95_1_21_, output shape: {[2048,7,7]}

            self.conv93_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_22_, output shape: {[32,7,7]}

            self.batchnorm93_1_22_ = gluon.nn.BatchNorm()
            # batchnorm93_1_22_, output shape: {[32,7,7]}

            self.relu93_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_22_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_22_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_22_, output shape: {[32,7,7]}

            self.batchnorm94_1_22_ = gluon.nn.BatchNorm()
            # batchnorm94_1_22_, output shape: {[32,7,7]}

            self.relu94_1_22_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_22_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_22_, output shape: {[2048,7,7]}

            self.batchnorm95_1_22_ = gluon.nn.BatchNorm()
            # batchnorm95_1_22_, output shape: {[2048,7,7]}

            self.conv93_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_23_, output shape: {[32,7,7]}

            self.batchnorm93_1_23_ = gluon.nn.BatchNorm()
            # batchnorm93_1_23_, output shape: {[32,7,7]}

            self.relu93_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_23_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_23_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_23_, output shape: {[32,7,7]}

            self.batchnorm94_1_23_ = gluon.nn.BatchNorm()
            # batchnorm94_1_23_, output shape: {[32,7,7]}

            self.relu94_1_23_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_23_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_23_, output shape: {[2048,7,7]}

            self.batchnorm95_1_23_ = gluon.nn.BatchNorm()
            # batchnorm95_1_23_, output shape: {[2048,7,7]}

            self.conv93_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_24_, output shape: {[32,7,7]}

            self.batchnorm93_1_24_ = gluon.nn.BatchNorm()
            # batchnorm93_1_24_, output shape: {[32,7,7]}

            self.relu93_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_24_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_24_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_24_, output shape: {[32,7,7]}

            self.batchnorm94_1_24_ = gluon.nn.BatchNorm()
            # batchnorm94_1_24_, output shape: {[32,7,7]}

            self.relu94_1_24_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_24_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_24_, output shape: {[2048,7,7]}

            self.batchnorm95_1_24_ = gluon.nn.BatchNorm()
            # batchnorm95_1_24_, output shape: {[2048,7,7]}

            self.conv93_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_25_, output shape: {[32,7,7]}

            self.batchnorm93_1_25_ = gluon.nn.BatchNorm()
            # batchnorm93_1_25_, output shape: {[32,7,7]}

            self.relu93_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_25_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_25_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_25_, output shape: {[32,7,7]}

            self.batchnorm94_1_25_ = gluon.nn.BatchNorm()
            # batchnorm94_1_25_, output shape: {[32,7,7]}

            self.relu94_1_25_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_25_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_25_, output shape: {[2048,7,7]}

            self.batchnorm95_1_25_ = gluon.nn.BatchNorm()
            # batchnorm95_1_25_, output shape: {[2048,7,7]}

            self.conv93_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_26_, output shape: {[32,7,7]}

            self.batchnorm93_1_26_ = gluon.nn.BatchNorm()
            # batchnorm93_1_26_, output shape: {[32,7,7]}

            self.relu93_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_26_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_26_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_26_, output shape: {[32,7,7]}

            self.batchnorm94_1_26_ = gluon.nn.BatchNorm()
            # batchnorm94_1_26_, output shape: {[32,7,7]}

            self.relu94_1_26_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_26_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_26_, output shape: {[2048,7,7]}

            self.batchnorm95_1_26_ = gluon.nn.BatchNorm()
            # batchnorm95_1_26_, output shape: {[2048,7,7]}

            self.conv93_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_27_, output shape: {[32,7,7]}

            self.batchnorm93_1_27_ = gluon.nn.BatchNorm()
            # batchnorm93_1_27_, output shape: {[32,7,7]}

            self.relu93_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_27_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_27_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_27_, output shape: {[32,7,7]}

            self.batchnorm94_1_27_ = gluon.nn.BatchNorm()
            # batchnorm94_1_27_, output shape: {[32,7,7]}

            self.relu94_1_27_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_27_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_27_, output shape: {[2048,7,7]}

            self.batchnorm95_1_27_ = gluon.nn.BatchNorm()
            # batchnorm95_1_27_, output shape: {[2048,7,7]}

            self.conv93_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_28_, output shape: {[32,7,7]}

            self.batchnorm93_1_28_ = gluon.nn.BatchNorm()
            # batchnorm93_1_28_, output shape: {[32,7,7]}

            self.relu93_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_28_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_28_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_28_, output shape: {[32,7,7]}

            self.batchnorm94_1_28_ = gluon.nn.BatchNorm()
            # batchnorm94_1_28_, output shape: {[32,7,7]}

            self.relu94_1_28_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_28_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_28_, output shape: {[2048,7,7]}

            self.batchnorm95_1_28_ = gluon.nn.BatchNorm()
            # batchnorm95_1_28_, output shape: {[2048,7,7]}

            self.conv93_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_29_, output shape: {[32,7,7]}

            self.batchnorm93_1_29_ = gluon.nn.BatchNorm()
            # batchnorm93_1_29_, output shape: {[32,7,7]}

            self.relu93_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_29_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_29_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_29_, output shape: {[32,7,7]}

            self.batchnorm94_1_29_ = gluon.nn.BatchNorm()
            # batchnorm94_1_29_, output shape: {[32,7,7]}

            self.relu94_1_29_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_29_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_29_, output shape: {[2048,7,7]}

            self.batchnorm95_1_29_ = gluon.nn.BatchNorm()
            # batchnorm95_1_29_, output shape: {[2048,7,7]}

            self.conv93_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_30_, output shape: {[32,7,7]}

            self.batchnorm93_1_30_ = gluon.nn.BatchNorm()
            # batchnorm93_1_30_, output shape: {[32,7,7]}

            self.relu93_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_30_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_30_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_30_, output shape: {[32,7,7]}

            self.batchnorm94_1_30_ = gluon.nn.BatchNorm()
            # batchnorm94_1_30_, output shape: {[32,7,7]}

            self.relu94_1_30_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_30_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_30_, output shape: {[2048,7,7]}

            self.batchnorm95_1_30_ = gluon.nn.BatchNorm()
            # batchnorm95_1_30_, output shape: {[2048,7,7]}

            self.conv93_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_31_, output shape: {[32,7,7]}

            self.batchnorm93_1_31_ = gluon.nn.BatchNorm()
            # batchnorm93_1_31_, output shape: {[32,7,7]}

            self.relu93_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_31_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_31_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_31_, output shape: {[32,7,7]}

            self.batchnorm94_1_31_ = gluon.nn.BatchNorm()
            # batchnorm94_1_31_, output shape: {[32,7,7]}

            self.relu94_1_31_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_31_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_31_, output shape: {[2048,7,7]}

            self.batchnorm95_1_31_ = gluon.nn.BatchNorm()
            # batchnorm95_1_31_, output shape: {[2048,7,7]}

            self.conv93_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv93_1_32_, output shape: {[32,7,7]}

            self.batchnorm93_1_32_ = gluon.nn.BatchNorm()
            # batchnorm93_1_32_, output shape: {[32,7,7]}

            self.relu93_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv94_1_32_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv94_1_32_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv94_1_32_, output shape: {[32,7,7]}

            self.batchnorm94_1_32_ = gluon.nn.BatchNorm()
            # batchnorm94_1_32_, output shape: {[32,7,7]}

            self.relu94_1_32_ = gluon.nn.Activation(activation='relu')
            self.conv95_1_32_ = gluon.nn.Conv2D(channels=2048,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv95_1_32_, output shape: {[2048,7,7]}

            self.batchnorm95_1_32_ = gluon.nn.BatchNorm()
            # batchnorm95_1_32_, output shape: {[2048,7,7]}

            self.relu97_ = gluon.nn.Activation(activation='relu')
            self.globalpooling97_ = gluon.nn.GlobalAvgPool2D()
            # globalpooling97_, output shape: {[2048,1,1]}

            self.fc97_ = gluon.nn.Dense(units=1000, use_bias=True, flatten=True)
            # fc97_, output shape: {[1000,1,1]}


            pass

    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)
        conv1_padding = self.conv1_padding(data_)
        conv1_ = self.conv1_(conv1_padding)
        batchnorm1_ = self.batchnorm1_(conv1_)
        relu1_ = self.relu1_(batchnorm1_)
        pool1_padding = self.pool1_padding(relu1_)
        pool1_ = self.pool1_(pool1_padding)
        conv3_1_1_ = self.conv3_1_1_(pool1_)
        batchnorm3_1_1_ = self.batchnorm3_1_1_(conv3_1_1_)
        relu3_1_1_ = self.relu3_1_1_(batchnorm3_1_1_)
        conv4_1_1_padding = self.conv4_1_1_padding(relu3_1_1_)
        conv4_1_1_ = self.conv4_1_1_(conv4_1_1_padding)
        batchnorm4_1_1_ = self.batchnorm4_1_1_(conv4_1_1_)
        relu4_1_1_ = self.relu4_1_1_(batchnorm4_1_1_)
        conv5_1_1_ = self.conv5_1_1_(relu4_1_1_)
        batchnorm5_1_1_ = self.batchnorm5_1_1_(conv5_1_1_)
        conv3_1_2_ = self.conv3_1_2_(pool1_)
        batchnorm3_1_2_ = self.batchnorm3_1_2_(conv3_1_2_)
        relu3_1_2_ = self.relu3_1_2_(batchnorm3_1_2_)
        conv4_1_2_padding = self.conv4_1_2_padding(relu3_1_2_)
        conv4_1_2_ = self.conv4_1_2_(conv4_1_2_padding)
        batchnorm4_1_2_ = self.batchnorm4_1_2_(conv4_1_2_)
        relu4_1_2_ = self.relu4_1_2_(batchnorm4_1_2_)
        conv5_1_2_ = self.conv5_1_2_(relu4_1_2_)
        batchnorm5_1_2_ = self.batchnorm5_1_2_(conv5_1_2_)
        conv3_1_3_ = self.conv3_1_3_(pool1_)
        batchnorm3_1_3_ = self.batchnorm3_1_3_(conv3_1_3_)
        relu3_1_3_ = self.relu3_1_3_(batchnorm3_1_3_)
        conv4_1_3_padding = self.conv4_1_3_padding(relu3_1_3_)
        conv4_1_3_ = self.conv4_1_3_(conv4_1_3_padding)
        batchnorm4_1_3_ = self.batchnorm4_1_3_(conv4_1_3_)
        relu4_1_3_ = self.relu4_1_3_(batchnorm4_1_3_)
        conv5_1_3_ = self.conv5_1_3_(relu4_1_3_)
        batchnorm5_1_3_ = self.batchnorm5_1_3_(conv5_1_3_)
        conv3_1_4_ = self.conv3_1_4_(pool1_)
        batchnorm3_1_4_ = self.batchnorm3_1_4_(conv3_1_4_)
        relu3_1_4_ = self.relu3_1_4_(batchnorm3_1_4_)
        conv4_1_4_padding = self.conv4_1_4_padding(relu3_1_4_)
        conv4_1_4_ = self.conv4_1_4_(conv4_1_4_padding)
        batchnorm4_1_4_ = self.batchnorm4_1_4_(conv4_1_4_)
        relu4_1_4_ = self.relu4_1_4_(batchnorm4_1_4_)
        conv5_1_4_ = self.conv5_1_4_(relu4_1_4_)
        batchnorm5_1_4_ = self.batchnorm5_1_4_(conv5_1_4_)
        conv3_1_5_ = self.conv3_1_5_(pool1_)
        batchnorm3_1_5_ = self.batchnorm3_1_5_(conv3_1_5_)
        relu3_1_5_ = self.relu3_1_5_(batchnorm3_1_5_)
        conv4_1_5_padding = self.conv4_1_5_padding(relu3_1_5_)
        conv4_1_5_ = self.conv4_1_5_(conv4_1_5_padding)
        batchnorm4_1_5_ = self.batchnorm4_1_5_(conv4_1_5_)
        relu4_1_5_ = self.relu4_1_5_(batchnorm4_1_5_)
        conv5_1_5_ = self.conv5_1_5_(relu4_1_5_)
        batchnorm5_1_5_ = self.batchnorm5_1_5_(conv5_1_5_)
        conv3_1_6_ = self.conv3_1_6_(pool1_)
        batchnorm3_1_6_ = self.batchnorm3_1_6_(conv3_1_6_)
        relu3_1_6_ = self.relu3_1_6_(batchnorm3_1_6_)
        conv4_1_6_padding = self.conv4_1_6_padding(relu3_1_6_)
        conv4_1_6_ = self.conv4_1_6_(conv4_1_6_padding)
        batchnorm4_1_6_ = self.batchnorm4_1_6_(conv4_1_6_)
        relu4_1_6_ = self.relu4_1_6_(batchnorm4_1_6_)
        conv5_1_6_ = self.conv5_1_6_(relu4_1_6_)
        batchnorm5_1_6_ = self.batchnorm5_1_6_(conv5_1_6_)
        conv3_1_7_ = self.conv3_1_7_(pool1_)
        batchnorm3_1_7_ = self.batchnorm3_1_7_(conv3_1_7_)
        relu3_1_7_ = self.relu3_1_7_(batchnorm3_1_7_)
        conv4_1_7_padding = self.conv4_1_7_padding(relu3_1_7_)
        conv4_1_7_ = self.conv4_1_7_(conv4_1_7_padding)
        batchnorm4_1_7_ = self.batchnorm4_1_7_(conv4_1_7_)
        relu4_1_7_ = self.relu4_1_7_(batchnorm4_1_7_)
        conv5_1_7_ = self.conv5_1_7_(relu4_1_7_)
        batchnorm5_1_7_ = self.batchnorm5_1_7_(conv5_1_7_)
        conv3_1_8_ = self.conv3_1_8_(pool1_)
        batchnorm3_1_8_ = self.batchnorm3_1_8_(conv3_1_8_)
        relu3_1_8_ = self.relu3_1_8_(batchnorm3_1_8_)
        conv4_1_8_padding = self.conv4_1_8_padding(relu3_1_8_)
        conv4_1_8_ = self.conv4_1_8_(conv4_1_8_padding)
        batchnorm4_1_8_ = self.batchnorm4_1_8_(conv4_1_8_)
        relu4_1_8_ = self.relu4_1_8_(batchnorm4_1_8_)
        conv5_1_8_ = self.conv5_1_8_(relu4_1_8_)
        batchnorm5_1_8_ = self.batchnorm5_1_8_(conv5_1_8_)
        conv3_1_9_ = self.conv3_1_9_(pool1_)
        batchnorm3_1_9_ = self.batchnorm3_1_9_(conv3_1_9_)
        relu3_1_9_ = self.relu3_1_9_(batchnorm3_1_9_)
        conv4_1_9_padding = self.conv4_1_9_padding(relu3_1_9_)
        conv4_1_9_ = self.conv4_1_9_(conv4_1_9_padding)
        batchnorm4_1_9_ = self.batchnorm4_1_9_(conv4_1_9_)
        relu4_1_9_ = self.relu4_1_9_(batchnorm4_1_9_)
        conv5_1_9_ = self.conv5_1_9_(relu4_1_9_)
        batchnorm5_1_9_ = self.batchnorm5_1_9_(conv5_1_9_)
        conv3_1_10_ = self.conv3_1_10_(pool1_)
        batchnorm3_1_10_ = self.batchnorm3_1_10_(conv3_1_10_)
        relu3_1_10_ = self.relu3_1_10_(batchnorm3_1_10_)
        conv4_1_10_padding = self.conv4_1_10_padding(relu3_1_10_)
        conv4_1_10_ = self.conv4_1_10_(conv4_1_10_padding)
        batchnorm4_1_10_ = self.batchnorm4_1_10_(conv4_1_10_)
        relu4_1_10_ = self.relu4_1_10_(batchnorm4_1_10_)
        conv5_1_10_ = self.conv5_1_10_(relu4_1_10_)
        batchnorm5_1_10_ = self.batchnorm5_1_10_(conv5_1_10_)
        conv3_1_11_ = self.conv3_1_11_(pool1_)
        batchnorm3_1_11_ = self.batchnorm3_1_11_(conv3_1_11_)
        relu3_1_11_ = self.relu3_1_11_(batchnorm3_1_11_)
        conv4_1_11_padding = self.conv4_1_11_padding(relu3_1_11_)
        conv4_1_11_ = self.conv4_1_11_(conv4_1_11_padding)
        batchnorm4_1_11_ = self.batchnorm4_1_11_(conv4_1_11_)
        relu4_1_11_ = self.relu4_1_11_(batchnorm4_1_11_)
        conv5_1_11_ = self.conv5_1_11_(relu4_1_11_)
        batchnorm5_1_11_ = self.batchnorm5_1_11_(conv5_1_11_)
        conv3_1_12_ = self.conv3_1_12_(pool1_)
        batchnorm3_1_12_ = self.batchnorm3_1_12_(conv3_1_12_)
        relu3_1_12_ = self.relu3_1_12_(batchnorm3_1_12_)
        conv4_1_12_padding = self.conv4_1_12_padding(relu3_1_12_)
        conv4_1_12_ = self.conv4_1_12_(conv4_1_12_padding)
        batchnorm4_1_12_ = self.batchnorm4_1_12_(conv4_1_12_)
        relu4_1_12_ = self.relu4_1_12_(batchnorm4_1_12_)
        conv5_1_12_ = self.conv5_1_12_(relu4_1_12_)
        batchnorm5_1_12_ = self.batchnorm5_1_12_(conv5_1_12_)
        conv3_1_13_ = self.conv3_1_13_(pool1_)
        batchnorm3_1_13_ = self.batchnorm3_1_13_(conv3_1_13_)
        relu3_1_13_ = self.relu3_1_13_(batchnorm3_1_13_)
        conv4_1_13_padding = self.conv4_1_13_padding(relu3_1_13_)
        conv4_1_13_ = self.conv4_1_13_(conv4_1_13_padding)
        batchnorm4_1_13_ = self.batchnorm4_1_13_(conv4_1_13_)
        relu4_1_13_ = self.relu4_1_13_(batchnorm4_1_13_)
        conv5_1_13_ = self.conv5_1_13_(relu4_1_13_)
        batchnorm5_1_13_ = self.batchnorm5_1_13_(conv5_1_13_)
        conv3_1_14_ = self.conv3_1_14_(pool1_)
        batchnorm3_1_14_ = self.batchnorm3_1_14_(conv3_1_14_)
        relu3_1_14_ = self.relu3_1_14_(batchnorm3_1_14_)
        conv4_1_14_padding = self.conv4_1_14_padding(relu3_1_14_)
        conv4_1_14_ = self.conv4_1_14_(conv4_1_14_padding)
        batchnorm4_1_14_ = self.batchnorm4_1_14_(conv4_1_14_)
        relu4_1_14_ = self.relu4_1_14_(batchnorm4_1_14_)
        conv5_1_14_ = self.conv5_1_14_(relu4_1_14_)
        batchnorm5_1_14_ = self.batchnorm5_1_14_(conv5_1_14_)
        conv3_1_15_ = self.conv3_1_15_(pool1_)
        batchnorm3_1_15_ = self.batchnorm3_1_15_(conv3_1_15_)
        relu3_1_15_ = self.relu3_1_15_(batchnorm3_1_15_)
        conv4_1_15_padding = self.conv4_1_15_padding(relu3_1_15_)
        conv4_1_15_ = self.conv4_1_15_(conv4_1_15_padding)
        batchnorm4_1_15_ = self.batchnorm4_1_15_(conv4_1_15_)
        relu4_1_15_ = self.relu4_1_15_(batchnorm4_1_15_)
        conv5_1_15_ = self.conv5_1_15_(relu4_1_15_)
        batchnorm5_1_15_ = self.batchnorm5_1_15_(conv5_1_15_)
        conv3_1_16_ = self.conv3_1_16_(pool1_)
        batchnorm3_1_16_ = self.batchnorm3_1_16_(conv3_1_16_)
        relu3_1_16_ = self.relu3_1_16_(batchnorm3_1_16_)
        conv4_1_16_padding = self.conv4_1_16_padding(relu3_1_16_)
        conv4_1_16_ = self.conv4_1_16_(conv4_1_16_padding)
        batchnorm4_1_16_ = self.batchnorm4_1_16_(conv4_1_16_)
        relu4_1_16_ = self.relu4_1_16_(batchnorm4_1_16_)
        conv5_1_16_ = self.conv5_1_16_(relu4_1_16_)
        batchnorm5_1_16_ = self.batchnorm5_1_16_(conv5_1_16_)
        conv3_1_17_ = self.conv3_1_17_(pool1_)
        batchnorm3_1_17_ = self.batchnorm3_1_17_(conv3_1_17_)
        relu3_1_17_ = self.relu3_1_17_(batchnorm3_1_17_)
        conv4_1_17_padding = self.conv4_1_17_padding(relu3_1_17_)
        conv4_1_17_ = self.conv4_1_17_(conv4_1_17_padding)
        batchnorm4_1_17_ = self.batchnorm4_1_17_(conv4_1_17_)
        relu4_1_17_ = self.relu4_1_17_(batchnorm4_1_17_)
        conv5_1_17_ = self.conv5_1_17_(relu4_1_17_)
        batchnorm5_1_17_ = self.batchnorm5_1_17_(conv5_1_17_)
        conv3_1_18_ = self.conv3_1_18_(pool1_)
        batchnorm3_1_18_ = self.batchnorm3_1_18_(conv3_1_18_)
        relu3_1_18_ = self.relu3_1_18_(batchnorm3_1_18_)
        conv4_1_18_padding = self.conv4_1_18_padding(relu3_1_18_)
        conv4_1_18_ = self.conv4_1_18_(conv4_1_18_padding)
        batchnorm4_1_18_ = self.batchnorm4_1_18_(conv4_1_18_)
        relu4_1_18_ = self.relu4_1_18_(batchnorm4_1_18_)
        conv5_1_18_ = self.conv5_1_18_(relu4_1_18_)
        batchnorm5_1_18_ = self.batchnorm5_1_18_(conv5_1_18_)
        conv3_1_19_ = self.conv3_1_19_(pool1_)
        batchnorm3_1_19_ = self.batchnorm3_1_19_(conv3_1_19_)
        relu3_1_19_ = self.relu3_1_19_(batchnorm3_1_19_)
        conv4_1_19_padding = self.conv4_1_19_padding(relu3_1_19_)
        conv4_1_19_ = self.conv4_1_19_(conv4_1_19_padding)
        batchnorm4_1_19_ = self.batchnorm4_1_19_(conv4_1_19_)
        relu4_1_19_ = self.relu4_1_19_(batchnorm4_1_19_)
        conv5_1_19_ = self.conv5_1_19_(relu4_1_19_)
        batchnorm5_1_19_ = self.batchnorm5_1_19_(conv5_1_19_)
        conv3_1_20_ = self.conv3_1_20_(pool1_)
        batchnorm3_1_20_ = self.batchnorm3_1_20_(conv3_1_20_)
        relu3_1_20_ = self.relu3_1_20_(batchnorm3_1_20_)
        conv4_1_20_padding = self.conv4_1_20_padding(relu3_1_20_)
        conv4_1_20_ = self.conv4_1_20_(conv4_1_20_padding)
        batchnorm4_1_20_ = self.batchnorm4_1_20_(conv4_1_20_)
        relu4_1_20_ = self.relu4_1_20_(batchnorm4_1_20_)
        conv5_1_20_ = self.conv5_1_20_(relu4_1_20_)
        batchnorm5_1_20_ = self.batchnorm5_1_20_(conv5_1_20_)
        conv3_1_21_ = self.conv3_1_21_(pool1_)
        batchnorm3_1_21_ = self.batchnorm3_1_21_(conv3_1_21_)
        relu3_1_21_ = self.relu3_1_21_(batchnorm3_1_21_)
        conv4_1_21_padding = self.conv4_1_21_padding(relu3_1_21_)
        conv4_1_21_ = self.conv4_1_21_(conv4_1_21_padding)
        batchnorm4_1_21_ = self.batchnorm4_1_21_(conv4_1_21_)
        relu4_1_21_ = self.relu4_1_21_(batchnorm4_1_21_)
        conv5_1_21_ = self.conv5_1_21_(relu4_1_21_)
        batchnorm5_1_21_ = self.batchnorm5_1_21_(conv5_1_21_)
        conv3_1_22_ = self.conv3_1_22_(pool1_)
        batchnorm3_1_22_ = self.batchnorm3_1_22_(conv3_1_22_)
        relu3_1_22_ = self.relu3_1_22_(batchnorm3_1_22_)
        conv4_1_22_padding = self.conv4_1_22_padding(relu3_1_22_)
        conv4_1_22_ = self.conv4_1_22_(conv4_1_22_padding)
        batchnorm4_1_22_ = self.batchnorm4_1_22_(conv4_1_22_)
        relu4_1_22_ = self.relu4_1_22_(batchnorm4_1_22_)
        conv5_1_22_ = self.conv5_1_22_(relu4_1_22_)
        batchnorm5_1_22_ = self.batchnorm5_1_22_(conv5_1_22_)
        conv3_1_23_ = self.conv3_1_23_(pool1_)
        batchnorm3_1_23_ = self.batchnorm3_1_23_(conv3_1_23_)
        relu3_1_23_ = self.relu3_1_23_(batchnorm3_1_23_)
        conv4_1_23_padding = self.conv4_1_23_padding(relu3_1_23_)
        conv4_1_23_ = self.conv4_1_23_(conv4_1_23_padding)
        batchnorm4_1_23_ = self.batchnorm4_1_23_(conv4_1_23_)
        relu4_1_23_ = self.relu4_1_23_(batchnorm4_1_23_)
        conv5_1_23_ = self.conv5_1_23_(relu4_1_23_)
        batchnorm5_1_23_ = self.batchnorm5_1_23_(conv5_1_23_)
        conv3_1_24_ = self.conv3_1_24_(pool1_)
        batchnorm3_1_24_ = self.batchnorm3_1_24_(conv3_1_24_)
        relu3_1_24_ = self.relu3_1_24_(batchnorm3_1_24_)
        conv4_1_24_padding = self.conv4_1_24_padding(relu3_1_24_)
        conv4_1_24_ = self.conv4_1_24_(conv4_1_24_padding)
        batchnorm4_1_24_ = self.batchnorm4_1_24_(conv4_1_24_)
        relu4_1_24_ = self.relu4_1_24_(batchnorm4_1_24_)
        conv5_1_24_ = self.conv5_1_24_(relu4_1_24_)
        batchnorm5_1_24_ = self.batchnorm5_1_24_(conv5_1_24_)
        conv3_1_25_ = self.conv3_1_25_(pool1_)
        batchnorm3_1_25_ = self.batchnorm3_1_25_(conv3_1_25_)
        relu3_1_25_ = self.relu3_1_25_(batchnorm3_1_25_)
        conv4_1_25_padding = self.conv4_1_25_padding(relu3_1_25_)
        conv4_1_25_ = self.conv4_1_25_(conv4_1_25_padding)
        batchnorm4_1_25_ = self.batchnorm4_1_25_(conv4_1_25_)
        relu4_1_25_ = self.relu4_1_25_(batchnorm4_1_25_)
        conv5_1_25_ = self.conv5_1_25_(relu4_1_25_)
        batchnorm5_1_25_ = self.batchnorm5_1_25_(conv5_1_25_)
        conv3_1_26_ = self.conv3_1_26_(pool1_)
        batchnorm3_1_26_ = self.batchnorm3_1_26_(conv3_1_26_)
        relu3_1_26_ = self.relu3_1_26_(batchnorm3_1_26_)
        conv4_1_26_padding = self.conv4_1_26_padding(relu3_1_26_)
        conv4_1_26_ = self.conv4_1_26_(conv4_1_26_padding)
        batchnorm4_1_26_ = self.batchnorm4_1_26_(conv4_1_26_)
        relu4_1_26_ = self.relu4_1_26_(batchnorm4_1_26_)
        conv5_1_26_ = self.conv5_1_26_(relu4_1_26_)
        batchnorm5_1_26_ = self.batchnorm5_1_26_(conv5_1_26_)
        conv3_1_27_ = self.conv3_1_27_(pool1_)
        batchnorm3_1_27_ = self.batchnorm3_1_27_(conv3_1_27_)
        relu3_1_27_ = self.relu3_1_27_(batchnorm3_1_27_)
        conv4_1_27_padding = self.conv4_1_27_padding(relu3_1_27_)
        conv4_1_27_ = self.conv4_1_27_(conv4_1_27_padding)
        batchnorm4_1_27_ = self.batchnorm4_1_27_(conv4_1_27_)
        relu4_1_27_ = self.relu4_1_27_(batchnorm4_1_27_)
        conv5_1_27_ = self.conv5_1_27_(relu4_1_27_)
        batchnorm5_1_27_ = self.batchnorm5_1_27_(conv5_1_27_)
        conv3_1_28_ = self.conv3_1_28_(pool1_)
        batchnorm3_1_28_ = self.batchnorm3_1_28_(conv3_1_28_)
        relu3_1_28_ = self.relu3_1_28_(batchnorm3_1_28_)
        conv4_1_28_padding = self.conv4_1_28_padding(relu3_1_28_)
        conv4_1_28_ = self.conv4_1_28_(conv4_1_28_padding)
        batchnorm4_1_28_ = self.batchnorm4_1_28_(conv4_1_28_)
        relu4_1_28_ = self.relu4_1_28_(batchnorm4_1_28_)
        conv5_1_28_ = self.conv5_1_28_(relu4_1_28_)
        batchnorm5_1_28_ = self.batchnorm5_1_28_(conv5_1_28_)
        conv3_1_29_ = self.conv3_1_29_(pool1_)
        batchnorm3_1_29_ = self.batchnorm3_1_29_(conv3_1_29_)
        relu3_1_29_ = self.relu3_1_29_(batchnorm3_1_29_)
        conv4_1_29_padding = self.conv4_1_29_padding(relu3_1_29_)
        conv4_1_29_ = self.conv4_1_29_(conv4_1_29_padding)
        batchnorm4_1_29_ = self.batchnorm4_1_29_(conv4_1_29_)
        relu4_1_29_ = self.relu4_1_29_(batchnorm4_1_29_)
        conv5_1_29_ = self.conv5_1_29_(relu4_1_29_)
        batchnorm5_1_29_ = self.batchnorm5_1_29_(conv5_1_29_)
        conv3_1_30_ = self.conv3_1_30_(pool1_)
        batchnorm3_1_30_ = self.batchnorm3_1_30_(conv3_1_30_)
        relu3_1_30_ = self.relu3_1_30_(batchnorm3_1_30_)
        conv4_1_30_padding = self.conv4_1_30_padding(relu3_1_30_)
        conv4_1_30_ = self.conv4_1_30_(conv4_1_30_padding)
        batchnorm4_1_30_ = self.batchnorm4_1_30_(conv4_1_30_)
        relu4_1_30_ = self.relu4_1_30_(batchnorm4_1_30_)
        conv5_1_30_ = self.conv5_1_30_(relu4_1_30_)
        batchnorm5_1_30_ = self.batchnorm5_1_30_(conv5_1_30_)
        conv3_1_31_ = self.conv3_1_31_(pool1_)
        batchnorm3_1_31_ = self.batchnorm3_1_31_(conv3_1_31_)
        relu3_1_31_ = self.relu3_1_31_(batchnorm3_1_31_)
        conv4_1_31_padding = self.conv4_1_31_padding(relu3_1_31_)
        conv4_1_31_ = self.conv4_1_31_(conv4_1_31_padding)
        batchnorm4_1_31_ = self.batchnorm4_1_31_(conv4_1_31_)
        relu4_1_31_ = self.relu4_1_31_(batchnorm4_1_31_)
        conv5_1_31_ = self.conv5_1_31_(relu4_1_31_)
        batchnorm5_1_31_ = self.batchnorm5_1_31_(conv5_1_31_)
        conv3_1_32_ = self.conv3_1_32_(pool1_)
        batchnorm3_1_32_ = self.batchnorm3_1_32_(conv3_1_32_)
        relu3_1_32_ = self.relu3_1_32_(batchnorm3_1_32_)
        conv4_1_32_padding = self.conv4_1_32_padding(relu3_1_32_)
        conv4_1_32_ = self.conv4_1_32_(conv4_1_32_padding)
        batchnorm4_1_32_ = self.batchnorm4_1_32_(conv4_1_32_)
        relu4_1_32_ = self.relu4_1_32_(batchnorm4_1_32_)
        conv5_1_32_ = self.conv5_1_32_(relu4_1_32_)
        batchnorm5_1_32_ = self.batchnorm5_1_32_(conv5_1_32_)
        add6_1_ = batchnorm5_1_1_ + batchnorm5_1_2_ + batchnorm5_1_3_ + batchnorm5_1_4_ + batchnorm5_1_5_ + batchnorm5_1_6_ + batchnorm5_1_7_ + batchnorm5_1_8_ + batchnorm5_1_9_ + batchnorm5_1_10_ + batchnorm5_1_11_ + batchnorm5_1_12_ + batchnorm5_1_13_ + batchnorm5_1_14_ + batchnorm5_1_15_ + batchnorm5_1_16_ + batchnorm5_1_17_ + batchnorm5_1_18_ + batchnorm5_1_19_ + batchnorm5_1_20_ + batchnorm5_1_21_ + batchnorm5_1_22_ + batchnorm5_1_23_ + batchnorm5_1_24_ + batchnorm5_1_25_ + batchnorm5_1_26_ + batchnorm5_1_27_ + batchnorm5_1_28_ + batchnorm5_1_29_ + batchnorm5_1_30_ + batchnorm5_1_31_ + batchnorm5_1_32_
        conv2_2_ = self.conv2_2_(pool1_)
        batchnorm2_2_ = self.batchnorm2_2_(conv2_2_)
        add7_ = add6_1_ + batchnorm2_2_
        relu7_ = self.relu7_(add7_)
        conv9_1_1_ = self.conv9_1_1_(relu7_)
        batchnorm9_1_1_ = self.batchnorm9_1_1_(conv9_1_1_)
        relu9_1_1_ = self.relu9_1_1_(batchnorm9_1_1_)
        conv10_1_1_padding = self.conv10_1_1_padding(relu9_1_1_)
        conv10_1_1_ = self.conv10_1_1_(conv10_1_1_padding)
        batchnorm10_1_1_ = self.batchnorm10_1_1_(conv10_1_1_)
        relu10_1_1_ = self.relu10_1_1_(batchnorm10_1_1_)
        conv11_1_1_ = self.conv11_1_1_(relu10_1_1_)
        batchnorm11_1_1_ = self.batchnorm11_1_1_(conv11_1_1_)
        conv9_1_2_ = self.conv9_1_2_(relu7_)
        batchnorm9_1_2_ = self.batchnorm9_1_2_(conv9_1_2_)
        relu9_1_2_ = self.relu9_1_2_(batchnorm9_1_2_)
        conv10_1_2_padding = self.conv10_1_2_padding(relu9_1_2_)
        conv10_1_2_ = self.conv10_1_2_(conv10_1_2_padding)
        batchnorm10_1_2_ = self.batchnorm10_1_2_(conv10_1_2_)
        relu10_1_2_ = self.relu10_1_2_(batchnorm10_1_2_)
        conv11_1_2_ = self.conv11_1_2_(relu10_1_2_)
        batchnorm11_1_2_ = self.batchnorm11_1_2_(conv11_1_2_)
        conv9_1_3_ = self.conv9_1_3_(relu7_)
        batchnorm9_1_3_ = self.batchnorm9_1_3_(conv9_1_3_)
        relu9_1_3_ = self.relu9_1_3_(batchnorm9_1_3_)
        conv10_1_3_padding = self.conv10_1_3_padding(relu9_1_3_)
        conv10_1_3_ = self.conv10_1_3_(conv10_1_3_padding)
        batchnorm10_1_3_ = self.batchnorm10_1_3_(conv10_1_3_)
        relu10_1_3_ = self.relu10_1_3_(batchnorm10_1_3_)
        conv11_1_3_ = self.conv11_1_3_(relu10_1_3_)
        batchnorm11_1_3_ = self.batchnorm11_1_3_(conv11_1_3_)
        conv9_1_4_ = self.conv9_1_4_(relu7_)
        batchnorm9_1_4_ = self.batchnorm9_1_4_(conv9_1_4_)
        relu9_1_4_ = self.relu9_1_4_(batchnorm9_1_4_)
        conv10_1_4_padding = self.conv10_1_4_padding(relu9_1_4_)
        conv10_1_4_ = self.conv10_1_4_(conv10_1_4_padding)
        batchnorm10_1_4_ = self.batchnorm10_1_4_(conv10_1_4_)
        relu10_1_4_ = self.relu10_1_4_(batchnorm10_1_4_)
        conv11_1_4_ = self.conv11_1_4_(relu10_1_4_)
        batchnorm11_1_4_ = self.batchnorm11_1_4_(conv11_1_4_)
        conv9_1_5_ = self.conv9_1_5_(relu7_)
        batchnorm9_1_5_ = self.batchnorm9_1_5_(conv9_1_5_)
        relu9_1_5_ = self.relu9_1_5_(batchnorm9_1_5_)
        conv10_1_5_padding = self.conv10_1_5_padding(relu9_1_5_)
        conv10_1_5_ = self.conv10_1_5_(conv10_1_5_padding)
        batchnorm10_1_5_ = self.batchnorm10_1_5_(conv10_1_5_)
        relu10_1_5_ = self.relu10_1_5_(batchnorm10_1_5_)
        conv11_1_5_ = self.conv11_1_5_(relu10_1_5_)
        batchnorm11_1_5_ = self.batchnorm11_1_5_(conv11_1_5_)
        conv9_1_6_ = self.conv9_1_6_(relu7_)
        batchnorm9_1_6_ = self.batchnorm9_1_6_(conv9_1_6_)
        relu9_1_6_ = self.relu9_1_6_(batchnorm9_1_6_)
        conv10_1_6_padding = self.conv10_1_6_padding(relu9_1_6_)
        conv10_1_6_ = self.conv10_1_6_(conv10_1_6_padding)
        batchnorm10_1_6_ = self.batchnorm10_1_6_(conv10_1_6_)
        relu10_1_6_ = self.relu10_1_6_(batchnorm10_1_6_)
        conv11_1_6_ = self.conv11_1_6_(relu10_1_6_)
        batchnorm11_1_6_ = self.batchnorm11_1_6_(conv11_1_6_)
        conv9_1_7_ = self.conv9_1_7_(relu7_)
        batchnorm9_1_7_ = self.batchnorm9_1_7_(conv9_1_7_)
        relu9_1_7_ = self.relu9_1_7_(batchnorm9_1_7_)
        conv10_1_7_padding = self.conv10_1_7_padding(relu9_1_7_)
        conv10_1_7_ = self.conv10_1_7_(conv10_1_7_padding)
        batchnorm10_1_7_ = self.batchnorm10_1_7_(conv10_1_7_)
        relu10_1_7_ = self.relu10_1_7_(batchnorm10_1_7_)
        conv11_1_7_ = self.conv11_1_7_(relu10_1_7_)
        batchnorm11_1_7_ = self.batchnorm11_1_7_(conv11_1_7_)
        conv9_1_8_ = self.conv9_1_8_(relu7_)
        batchnorm9_1_8_ = self.batchnorm9_1_8_(conv9_1_8_)
        relu9_1_8_ = self.relu9_1_8_(batchnorm9_1_8_)
        conv10_1_8_padding = self.conv10_1_8_padding(relu9_1_8_)
        conv10_1_8_ = self.conv10_1_8_(conv10_1_8_padding)
        batchnorm10_1_8_ = self.batchnorm10_1_8_(conv10_1_8_)
        relu10_1_8_ = self.relu10_1_8_(batchnorm10_1_8_)
        conv11_1_8_ = self.conv11_1_8_(relu10_1_8_)
        batchnorm11_1_8_ = self.batchnorm11_1_8_(conv11_1_8_)
        conv9_1_9_ = self.conv9_1_9_(relu7_)
        batchnorm9_1_9_ = self.batchnorm9_1_9_(conv9_1_9_)
        relu9_1_9_ = self.relu9_1_9_(batchnorm9_1_9_)
        conv10_1_9_padding = self.conv10_1_9_padding(relu9_1_9_)
        conv10_1_9_ = self.conv10_1_9_(conv10_1_9_padding)
        batchnorm10_1_9_ = self.batchnorm10_1_9_(conv10_1_9_)
        relu10_1_9_ = self.relu10_1_9_(batchnorm10_1_9_)
        conv11_1_9_ = self.conv11_1_9_(relu10_1_9_)
        batchnorm11_1_9_ = self.batchnorm11_1_9_(conv11_1_9_)
        conv9_1_10_ = self.conv9_1_10_(relu7_)
        batchnorm9_1_10_ = self.batchnorm9_1_10_(conv9_1_10_)
        relu9_1_10_ = self.relu9_1_10_(batchnorm9_1_10_)
        conv10_1_10_padding = self.conv10_1_10_padding(relu9_1_10_)
        conv10_1_10_ = self.conv10_1_10_(conv10_1_10_padding)
        batchnorm10_1_10_ = self.batchnorm10_1_10_(conv10_1_10_)
        relu10_1_10_ = self.relu10_1_10_(batchnorm10_1_10_)
        conv11_1_10_ = self.conv11_1_10_(relu10_1_10_)
        batchnorm11_1_10_ = self.batchnorm11_1_10_(conv11_1_10_)
        conv9_1_11_ = self.conv9_1_11_(relu7_)
        batchnorm9_1_11_ = self.batchnorm9_1_11_(conv9_1_11_)
        relu9_1_11_ = self.relu9_1_11_(batchnorm9_1_11_)
        conv10_1_11_padding = self.conv10_1_11_padding(relu9_1_11_)
        conv10_1_11_ = self.conv10_1_11_(conv10_1_11_padding)
        batchnorm10_1_11_ = self.batchnorm10_1_11_(conv10_1_11_)
        relu10_1_11_ = self.relu10_1_11_(batchnorm10_1_11_)
        conv11_1_11_ = self.conv11_1_11_(relu10_1_11_)
        batchnorm11_1_11_ = self.batchnorm11_1_11_(conv11_1_11_)
        conv9_1_12_ = self.conv9_1_12_(relu7_)
        batchnorm9_1_12_ = self.batchnorm9_1_12_(conv9_1_12_)
        relu9_1_12_ = self.relu9_1_12_(batchnorm9_1_12_)
        conv10_1_12_padding = self.conv10_1_12_padding(relu9_1_12_)
        conv10_1_12_ = self.conv10_1_12_(conv10_1_12_padding)
        batchnorm10_1_12_ = self.batchnorm10_1_12_(conv10_1_12_)
        relu10_1_12_ = self.relu10_1_12_(batchnorm10_1_12_)
        conv11_1_12_ = self.conv11_1_12_(relu10_1_12_)
        batchnorm11_1_12_ = self.batchnorm11_1_12_(conv11_1_12_)
        conv9_1_13_ = self.conv9_1_13_(relu7_)
        batchnorm9_1_13_ = self.batchnorm9_1_13_(conv9_1_13_)
        relu9_1_13_ = self.relu9_1_13_(batchnorm9_1_13_)
        conv10_1_13_padding = self.conv10_1_13_padding(relu9_1_13_)
        conv10_1_13_ = self.conv10_1_13_(conv10_1_13_padding)
        batchnorm10_1_13_ = self.batchnorm10_1_13_(conv10_1_13_)
        relu10_1_13_ = self.relu10_1_13_(batchnorm10_1_13_)
        conv11_1_13_ = self.conv11_1_13_(relu10_1_13_)
        batchnorm11_1_13_ = self.batchnorm11_1_13_(conv11_1_13_)
        conv9_1_14_ = self.conv9_1_14_(relu7_)
        batchnorm9_1_14_ = self.batchnorm9_1_14_(conv9_1_14_)
        relu9_1_14_ = self.relu9_1_14_(batchnorm9_1_14_)
        conv10_1_14_padding = self.conv10_1_14_padding(relu9_1_14_)
        conv10_1_14_ = self.conv10_1_14_(conv10_1_14_padding)
        batchnorm10_1_14_ = self.batchnorm10_1_14_(conv10_1_14_)
        relu10_1_14_ = self.relu10_1_14_(batchnorm10_1_14_)
        conv11_1_14_ = self.conv11_1_14_(relu10_1_14_)
        batchnorm11_1_14_ = self.batchnorm11_1_14_(conv11_1_14_)
        conv9_1_15_ = self.conv9_1_15_(relu7_)
        batchnorm9_1_15_ = self.batchnorm9_1_15_(conv9_1_15_)
        relu9_1_15_ = self.relu9_1_15_(batchnorm9_1_15_)
        conv10_1_15_padding = self.conv10_1_15_padding(relu9_1_15_)
        conv10_1_15_ = self.conv10_1_15_(conv10_1_15_padding)
        batchnorm10_1_15_ = self.batchnorm10_1_15_(conv10_1_15_)
        relu10_1_15_ = self.relu10_1_15_(batchnorm10_1_15_)
        conv11_1_15_ = self.conv11_1_15_(relu10_1_15_)
        batchnorm11_1_15_ = self.batchnorm11_1_15_(conv11_1_15_)
        conv9_1_16_ = self.conv9_1_16_(relu7_)
        batchnorm9_1_16_ = self.batchnorm9_1_16_(conv9_1_16_)
        relu9_1_16_ = self.relu9_1_16_(batchnorm9_1_16_)
        conv10_1_16_padding = self.conv10_1_16_padding(relu9_1_16_)
        conv10_1_16_ = self.conv10_1_16_(conv10_1_16_padding)
        batchnorm10_1_16_ = self.batchnorm10_1_16_(conv10_1_16_)
        relu10_1_16_ = self.relu10_1_16_(batchnorm10_1_16_)
        conv11_1_16_ = self.conv11_1_16_(relu10_1_16_)
        batchnorm11_1_16_ = self.batchnorm11_1_16_(conv11_1_16_)
        conv9_1_17_ = self.conv9_1_17_(relu7_)
        batchnorm9_1_17_ = self.batchnorm9_1_17_(conv9_1_17_)
        relu9_1_17_ = self.relu9_1_17_(batchnorm9_1_17_)
        conv10_1_17_padding = self.conv10_1_17_padding(relu9_1_17_)
        conv10_1_17_ = self.conv10_1_17_(conv10_1_17_padding)
        batchnorm10_1_17_ = self.batchnorm10_1_17_(conv10_1_17_)
        relu10_1_17_ = self.relu10_1_17_(batchnorm10_1_17_)
        conv11_1_17_ = self.conv11_1_17_(relu10_1_17_)
        batchnorm11_1_17_ = self.batchnorm11_1_17_(conv11_1_17_)
        conv9_1_18_ = self.conv9_1_18_(relu7_)
        batchnorm9_1_18_ = self.batchnorm9_1_18_(conv9_1_18_)
        relu9_1_18_ = self.relu9_1_18_(batchnorm9_1_18_)
        conv10_1_18_padding = self.conv10_1_18_padding(relu9_1_18_)
        conv10_1_18_ = self.conv10_1_18_(conv10_1_18_padding)
        batchnorm10_1_18_ = self.batchnorm10_1_18_(conv10_1_18_)
        relu10_1_18_ = self.relu10_1_18_(batchnorm10_1_18_)
        conv11_1_18_ = self.conv11_1_18_(relu10_1_18_)
        batchnorm11_1_18_ = self.batchnorm11_1_18_(conv11_1_18_)
        conv9_1_19_ = self.conv9_1_19_(relu7_)
        batchnorm9_1_19_ = self.batchnorm9_1_19_(conv9_1_19_)
        relu9_1_19_ = self.relu9_1_19_(batchnorm9_1_19_)
        conv10_1_19_padding = self.conv10_1_19_padding(relu9_1_19_)
        conv10_1_19_ = self.conv10_1_19_(conv10_1_19_padding)
        batchnorm10_1_19_ = self.batchnorm10_1_19_(conv10_1_19_)
        relu10_1_19_ = self.relu10_1_19_(batchnorm10_1_19_)
        conv11_1_19_ = self.conv11_1_19_(relu10_1_19_)
        batchnorm11_1_19_ = self.batchnorm11_1_19_(conv11_1_19_)
        conv9_1_20_ = self.conv9_1_20_(relu7_)
        batchnorm9_1_20_ = self.batchnorm9_1_20_(conv9_1_20_)
        relu9_1_20_ = self.relu9_1_20_(batchnorm9_1_20_)
        conv10_1_20_padding = self.conv10_1_20_padding(relu9_1_20_)
        conv10_1_20_ = self.conv10_1_20_(conv10_1_20_padding)
        batchnorm10_1_20_ = self.batchnorm10_1_20_(conv10_1_20_)
        relu10_1_20_ = self.relu10_1_20_(batchnorm10_1_20_)
        conv11_1_20_ = self.conv11_1_20_(relu10_1_20_)
        batchnorm11_1_20_ = self.batchnorm11_1_20_(conv11_1_20_)
        conv9_1_21_ = self.conv9_1_21_(relu7_)
        batchnorm9_1_21_ = self.batchnorm9_1_21_(conv9_1_21_)
        relu9_1_21_ = self.relu9_1_21_(batchnorm9_1_21_)
        conv10_1_21_padding = self.conv10_1_21_padding(relu9_1_21_)
        conv10_1_21_ = self.conv10_1_21_(conv10_1_21_padding)
        batchnorm10_1_21_ = self.batchnorm10_1_21_(conv10_1_21_)
        relu10_1_21_ = self.relu10_1_21_(batchnorm10_1_21_)
        conv11_1_21_ = self.conv11_1_21_(relu10_1_21_)
        batchnorm11_1_21_ = self.batchnorm11_1_21_(conv11_1_21_)
        conv9_1_22_ = self.conv9_1_22_(relu7_)
        batchnorm9_1_22_ = self.batchnorm9_1_22_(conv9_1_22_)
        relu9_1_22_ = self.relu9_1_22_(batchnorm9_1_22_)
        conv10_1_22_padding = self.conv10_1_22_padding(relu9_1_22_)
        conv10_1_22_ = self.conv10_1_22_(conv10_1_22_padding)
        batchnorm10_1_22_ = self.batchnorm10_1_22_(conv10_1_22_)
        relu10_1_22_ = self.relu10_1_22_(batchnorm10_1_22_)
        conv11_1_22_ = self.conv11_1_22_(relu10_1_22_)
        batchnorm11_1_22_ = self.batchnorm11_1_22_(conv11_1_22_)
        conv9_1_23_ = self.conv9_1_23_(relu7_)
        batchnorm9_1_23_ = self.batchnorm9_1_23_(conv9_1_23_)
        relu9_1_23_ = self.relu9_1_23_(batchnorm9_1_23_)
        conv10_1_23_padding = self.conv10_1_23_padding(relu9_1_23_)
        conv10_1_23_ = self.conv10_1_23_(conv10_1_23_padding)
        batchnorm10_1_23_ = self.batchnorm10_1_23_(conv10_1_23_)
        relu10_1_23_ = self.relu10_1_23_(batchnorm10_1_23_)
        conv11_1_23_ = self.conv11_1_23_(relu10_1_23_)
        batchnorm11_1_23_ = self.batchnorm11_1_23_(conv11_1_23_)
        conv9_1_24_ = self.conv9_1_24_(relu7_)
        batchnorm9_1_24_ = self.batchnorm9_1_24_(conv9_1_24_)
        relu9_1_24_ = self.relu9_1_24_(batchnorm9_1_24_)
        conv10_1_24_padding = self.conv10_1_24_padding(relu9_1_24_)
        conv10_1_24_ = self.conv10_1_24_(conv10_1_24_padding)
        batchnorm10_1_24_ = self.batchnorm10_1_24_(conv10_1_24_)
        relu10_1_24_ = self.relu10_1_24_(batchnorm10_1_24_)
        conv11_1_24_ = self.conv11_1_24_(relu10_1_24_)
        batchnorm11_1_24_ = self.batchnorm11_1_24_(conv11_1_24_)
        conv9_1_25_ = self.conv9_1_25_(relu7_)
        batchnorm9_1_25_ = self.batchnorm9_1_25_(conv9_1_25_)
        relu9_1_25_ = self.relu9_1_25_(batchnorm9_1_25_)
        conv10_1_25_padding = self.conv10_1_25_padding(relu9_1_25_)
        conv10_1_25_ = self.conv10_1_25_(conv10_1_25_padding)
        batchnorm10_1_25_ = self.batchnorm10_1_25_(conv10_1_25_)
        relu10_1_25_ = self.relu10_1_25_(batchnorm10_1_25_)
        conv11_1_25_ = self.conv11_1_25_(relu10_1_25_)
        batchnorm11_1_25_ = self.batchnorm11_1_25_(conv11_1_25_)
        conv9_1_26_ = self.conv9_1_26_(relu7_)
        batchnorm9_1_26_ = self.batchnorm9_1_26_(conv9_1_26_)
        relu9_1_26_ = self.relu9_1_26_(batchnorm9_1_26_)
        conv10_1_26_padding = self.conv10_1_26_padding(relu9_1_26_)
        conv10_1_26_ = self.conv10_1_26_(conv10_1_26_padding)
        batchnorm10_1_26_ = self.batchnorm10_1_26_(conv10_1_26_)
        relu10_1_26_ = self.relu10_1_26_(batchnorm10_1_26_)
        conv11_1_26_ = self.conv11_1_26_(relu10_1_26_)
        batchnorm11_1_26_ = self.batchnorm11_1_26_(conv11_1_26_)
        conv9_1_27_ = self.conv9_1_27_(relu7_)
        batchnorm9_1_27_ = self.batchnorm9_1_27_(conv9_1_27_)
        relu9_1_27_ = self.relu9_1_27_(batchnorm9_1_27_)
        conv10_1_27_padding = self.conv10_1_27_padding(relu9_1_27_)
        conv10_1_27_ = self.conv10_1_27_(conv10_1_27_padding)
        batchnorm10_1_27_ = self.batchnorm10_1_27_(conv10_1_27_)
        relu10_1_27_ = self.relu10_1_27_(batchnorm10_1_27_)
        conv11_1_27_ = self.conv11_1_27_(relu10_1_27_)
        batchnorm11_1_27_ = self.batchnorm11_1_27_(conv11_1_27_)
        conv9_1_28_ = self.conv9_1_28_(relu7_)
        batchnorm9_1_28_ = self.batchnorm9_1_28_(conv9_1_28_)
        relu9_1_28_ = self.relu9_1_28_(batchnorm9_1_28_)
        conv10_1_28_padding = self.conv10_1_28_padding(relu9_1_28_)
        conv10_1_28_ = self.conv10_1_28_(conv10_1_28_padding)
        batchnorm10_1_28_ = self.batchnorm10_1_28_(conv10_1_28_)
        relu10_1_28_ = self.relu10_1_28_(batchnorm10_1_28_)
        conv11_1_28_ = self.conv11_1_28_(relu10_1_28_)
        batchnorm11_1_28_ = self.batchnorm11_1_28_(conv11_1_28_)
        conv9_1_29_ = self.conv9_1_29_(relu7_)
        batchnorm9_1_29_ = self.batchnorm9_1_29_(conv9_1_29_)
        relu9_1_29_ = self.relu9_1_29_(batchnorm9_1_29_)
        conv10_1_29_padding = self.conv10_1_29_padding(relu9_1_29_)
        conv10_1_29_ = self.conv10_1_29_(conv10_1_29_padding)
        batchnorm10_1_29_ = self.batchnorm10_1_29_(conv10_1_29_)
        relu10_1_29_ = self.relu10_1_29_(batchnorm10_1_29_)
        conv11_1_29_ = self.conv11_1_29_(relu10_1_29_)
        batchnorm11_1_29_ = self.batchnorm11_1_29_(conv11_1_29_)
        conv9_1_30_ = self.conv9_1_30_(relu7_)
        batchnorm9_1_30_ = self.batchnorm9_1_30_(conv9_1_30_)
        relu9_1_30_ = self.relu9_1_30_(batchnorm9_1_30_)
        conv10_1_30_padding = self.conv10_1_30_padding(relu9_1_30_)
        conv10_1_30_ = self.conv10_1_30_(conv10_1_30_padding)
        batchnorm10_1_30_ = self.batchnorm10_1_30_(conv10_1_30_)
        relu10_1_30_ = self.relu10_1_30_(batchnorm10_1_30_)
        conv11_1_30_ = self.conv11_1_30_(relu10_1_30_)
        batchnorm11_1_30_ = self.batchnorm11_1_30_(conv11_1_30_)
        conv9_1_31_ = self.conv9_1_31_(relu7_)
        batchnorm9_1_31_ = self.batchnorm9_1_31_(conv9_1_31_)
        relu9_1_31_ = self.relu9_1_31_(batchnorm9_1_31_)
        conv10_1_31_padding = self.conv10_1_31_padding(relu9_1_31_)
        conv10_1_31_ = self.conv10_1_31_(conv10_1_31_padding)
        batchnorm10_1_31_ = self.batchnorm10_1_31_(conv10_1_31_)
        relu10_1_31_ = self.relu10_1_31_(batchnorm10_1_31_)
        conv11_1_31_ = self.conv11_1_31_(relu10_1_31_)
        batchnorm11_1_31_ = self.batchnorm11_1_31_(conv11_1_31_)
        conv9_1_32_ = self.conv9_1_32_(relu7_)
        batchnorm9_1_32_ = self.batchnorm9_1_32_(conv9_1_32_)
        relu9_1_32_ = self.relu9_1_32_(batchnorm9_1_32_)
        conv10_1_32_padding = self.conv10_1_32_padding(relu9_1_32_)
        conv10_1_32_ = self.conv10_1_32_(conv10_1_32_padding)
        batchnorm10_1_32_ = self.batchnorm10_1_32_(conv10_1_32_)
        relu10_1_32_ = self.relu10_1_32_(batchnorm10_1_32_)
        conv11_1_32_ = self.conv11_1_32_(relu10_1_32_)
        batchnorm11_1_32_ = self.batchnorm11_1_32_(conv11_1_32_)
        add12_1_ = batchnorm11_1_1_ + batchnorm11_1_2_ + batchnorm11_1_3_ + batchnorm11_1_4_ + batchnorm11_1_5_ + batchnorm11_1_6_ + batchnorm11_1_7_ + batchnorm11_1_8_ + batchnorm11_1_9_ + batchnorm11_1_10_ + batchnorm11_1_11_ + batchnorm11_1_12_ + batchnorm11_1_13_ + batchnorm11_1_14_ + batchnorm11_1_15_ + batchnorm11_1_16_ + batchnorm11_1_17_ + batchnorm11_1_18_ + batchnorm11_1_19_ + batchnorm11_1_20_ + batchnorm11_1_21_ + batchnorm11_1_22_ + batchnorm11_1_23_ + batchnorm11_1_24_ + batchnorm11_1_25_ + batchnorm11_1_26_ + batchnorm11_1_27_ + batchnorm11_1_28_ + batchnorm11_1_29_ + batchnorm11_1_30_ + batchnorm11_1_31_ + batchnorm11_1_32_
        add13_ = add12_1_ + relu7_
        relu13_ = self.relu13_(add13_)
        conv15_1_1_ = self.conv15_1_1_(relu13_)
        batchnorm15_1_1_ = self.batchnorm15_1_1_(conv15_1_1_)
        relu15_1_1_ = self.relu15_1_1_(batchnorm15_1_1_)
        conv16_1_1_padding = self.conv16_1_1_padding(relu15_1_1_)
        conv16_1_1_ = self.conv16_1_1_(conv16_1_1_padding)
        batchnorm16_1_1_ = self.batchnorm16_1_1_(conv16_1_1_)
        relu16_1_1_ = self.relu16_1_1_(batchnorm16_1_1_)
        conv17_1_1_ = self.conv17_1_1_(relu16_1_1_)
        batchnorm17_1_1_ = self.batchnorm17_1_1_(conv17_1_1_)
        conv15_1_2_ = self.conv15_1_2_(relu13_)
        batchnorm15_1_2_ = self.batchnorm15_1_2_(conv15_1_2_)
        relu15_1_2_ = self.relu15_1_2_(batchnorm15_1_2_)
        conv16_1_2_padding = self.conv16_1_2_padding(relu15_1_2_)
        conv16_1_2_ = self.conv16_1_2_(conv16_1_2_padding)
        batchnorm16_1_2_ = self.batchnorm16_1_2_(conv16_1_2_)
        relu16_1_2_ = self.relu16_1_2_(batchnorm16_1_2_)
        conv17_1_2_ = self.conv17_1_2_(relu16_1_2_)
        batchnorm17_1_2_ = self.batchnorm17_1_2_(conv17_1_2_)
        conv15_1_3_ = self.conv15_1_3_(relu13_)
        batchnorm15_1_3_ = self.batchnorm15_1_3_(conv15_1_3_)
        relu15_1_3_ = self.relu15_1_3_(batchnorm15_1_3_)
        conv16_1_3_padding = self.conv16_1_3_padding(relu15_1_3_)
        conv16_1_3_ = self.conv16_1_3_(conv16_1_3_padding)
        batchnorm16_1_3_ = self.batchnorm16_1_3_(conv16_1_3_)
        relu16_1_3_ = self.relu16_1_3_(batchnorm16_1_3_)
        conv17_1_3_ = self.conv17_1_3_(relu16_1_3_)
        batchnorm17_1_3_ = self.batchnorm17_1_3_(conv17_1_3_)
        conv15_1_4_ = self.conv15_1_4_(relu13_)
        batchnorm15_1_4_ = self.batchnorm15_1_4_(conv15_1_4_)
        relu15_1_4_ = self.relu15_1_4_(batchnorm15_1_4_)
        conv16_1_4_padding = self.conv16_1_4_padding(relu15_1_4_)
        conv16_1_4_ = self.conv16_1_4_(conv16_1_4_padding)
        batchnorm16_1_4_ = self.batchnorm16_1_4_(conv16_1_4_)
        relu16_1_4_ = self.relu16_1_4_(batchnorm16_1_4_)
        conv17_1_4_ = self.conv17_1_4_(relu16_1_4_)
        batchnorm17_1_4_ = self.batchnorm17_1_4_(conv17_1_4_)
        conv15_1_5_ = self.conv15_1_5_(relu13_)
        batchnorm15_1_5_ = self.batchnorm15_1_5_(conv15_1_5_)
        relu15_1_5_ = self.relu15_1_5_(batchnorm15_1_5_)
        conv16_1_5_padding = self.conv16_1_5_padding(relu15_1_5_)
        conv16_1_5_ = self.conv16_1_5_(conv16_1_5_padding)
        batchnorm16_1_5_ = self.batchnorm16_1_5_(conv16_1_5_)
        relu16_1_5_ = self.relu16_1_5_(batchnorm16_1_5_)
        conv17_1_5_ = self.conv17_1_5_(relu16_1_5_)
        batchnorm17_1_5_ = self.batchnorm17_1_5_(conv17_1_5_)
        conv15_1_6_ = self.conv15_1_6_(relu13_)
        batchnorm15_1_6_ = self.batchnorm15_1_6_(conv15_1_6_)
        relu15_1_6_ = self.relu15_1_6_(batchnorm15_1_6_)
        conv16_1_6_padding = self.conv16_1_6_padding(relu15_1_6_)
        conv16_1_6_ = self.conv16_1_6_(conv16_1_6_padding)
        batchnorm16_1_6_ = self.batchnorm16_1_6_(conv16_1_6_)
        relu16_1_6_ = self.relu16_1_6_(batchnorm16_1_6_)
        conv17_1_6_ = self.conv17_1_6_(relu16_1_6_)
        batchnorm17_1_6_ = self.batchnorm17_1_6_(conv17_1_6_)
        conv15_1_7_ = self.conv15_1_7_(relu13_)
        batchnorm15_1_7_ = self.batchnorm15_1_7_(conv15_1_7_)
        relu15_1_7_ = self.relu15_1_7_(batchnorm15_1_7_)
        conv16_1_7_padding = self.conv16_1_7_padding(relu15_1_7_)
        conv16_1_7_ = self.conv16_1_7_(conv16_1_7_padding)
        batchnorm16_1_7_ = self.batchnorm16_1_7_(conv16_1_7_)
        relu16_1_7_ = self.relu16_1_7_(batchnorm16_1_7_)
        conv17_1_7_ = self.conv17_1_7_(relu16_1_7_)
        batchnorm17_1_7_ = self.batchnorm17_1_7_(conv17_1_7_)
        conv15_1_8_ = self.conv15_1_8_(relu13_)
        batchnorm15_1_8_ = self.batchnorm15_1_8_(conv15_1_8_)
        relu15_1_8_ = self.relu15_1_8_(batchnorm15_1_8_)
        conv16_1_8_padding = self.conv16_1_8_padding(relu15_1_8_)
        conv16_1_8_ = self.conv16_1_8_(conv16_1_8_padding)
        batchnorm16_1_8_ = self.batchnorm16_1_8_(conv16_1_8_)
        relu16_1_8_ = self.relu16_1_8_(batchnorm16_1_8_)
        conv17_1_8_ = self.conv17_1_8_(relu16_1_8_)
        batchnorm17_1_8_ = self.batchnorm17_1_8_(conv17_1_8_)
        conv15_1_9_ = self.conv15_1_9_(relu13_)
        batchnorm15_1_9_ = self.batchnorm15_1_9_(conv15_1_9_)
        relu15_1_9_ = self.relu15_1_9_(batchnorm15_1_9_)
        conv16_1_9_padding = self.conv16_1_9_padding(relu15_1_9_)
        conv16_1_9_ = self.conv16_1_9_(conv16_1_9_padding)
        batchnorm16_1_9_ = self.batchnorm16_1_9_(conv16_1_9_)
        relu16_1_9_ = self.relu16_1_9_(batchnorm16_1_9_)
        conv17_1_9_ = self.conv17_1_9_(relu16_1_9_)
        batchnorm17_1_9_ = self.batchnorm17_1_9_(conv17_1_9_)
        conv15_1_10_ = self.conv15_1_10_(relu13_)
        batchnorm15_1_10_ = self.batchnorm15_1_10_(conv15_1_10_)
        relu15_1_10_ = self.relu15_1_10_(batchnorm15_1_10_)
        conv16_1_10_padding = self.conv16_1_10_padding(relu15_1_10_)
        conv16_1_10_ = self.conv16_1_10_(conv16_1_10_padding)
        batchnorm16_1_10_ = self.batchnorm16_1_10_(conv16_1_10_)
        relu16_1_10_ = self.relu16_1_10_(batchnorm16_1_10_)
        conv17_1_10_ = self.conv17_1_10_(relu16_1_10_)
        batchnorm17_1_10_ = self.batchnorm17_1_10_(conv17_1_10_)
        conv15_1_11_ = self.conv15_1_11_(relu13_)
        batchnorm15_1_11_ = self.batchnorm15_1_11_(conv15_1_11_)
        relu15_1_11_ = self.relu15_1_11_(batchnorm15_1_11_)
        conv16_1_11_padding = self.conv16_1_11_padding(relu15_1_11_)
        conv16_1_11_ = self.conv16_1_11_(conv16_1_11_padding)
        batchnorm16_1_11_ = self.batchnorm16_1_11_(conv16_1_11_)
        relu16_1_11_ = self.relu16_1_11_(batchnorm16_1_11_)
        conv17_1_11_ = self.conv17_1_11_(relu16_1_11_)
        batchnorm17_1_11_ = self.batchnorm17_1_11_(conv17_1_11_)
        conv15_1_12_ = self.conv15_1_12_(relu13_)
        batchnorm15_1_12_ = self.batchnorm15_1_12_(conv15_1_12_)
        relu15_1_12_ = self.relu15_1_12_(batchnorm15_1_12_)
        conv16_1_12_padding = self.conv16_1_12_padding(relu15_1_12_)
        conv16_1_12_ = self.conv16_1_12_(conv16_1_12_padding)
        batchnorm16_1_12_ = self.batchnorm16_1_12_(conv16_1_12_)
        relu16_1_12_ = self.relu16_1_12_(batchnorm16_1_12_)
        conv17_1_12_ = self.conv17_1_12_(relu16_1_12_)
        batchnorm17_1_12_ = self.batchnorm17_1_12_(conv17_1_12_)
        conv15_1_13_ = self.conv15_1_13_(relu13_)
        batchnorm15_1_13_ = self.batchnorm15_1_13_(conv15_1_13_)
        relu15_1_13_ = self.relu15_1_13_(batchnorm15_1_13_)
        conv16_1_13_padding = self.conv16_1_13_padding(relu15_1_13_)
        conv16_1_13_ = self.conv16_1_13_(conv16_1_13_padding)
        batchnorm16_1_13_ = self.batchnorm16_1_13_(conv16_1_13_)
        relu16_1_13_ = self.relu16_1_13_(batchnorm16_1_13_)
        conv17_1_13_ = self.conv17_1_13_(relu16_1_13_)
        batchnorm17_1_13_ = self.batchnorm17_1_13_(conv17_1_13_)
        conv15_1_14_ = self.conv15_1_14_(relu13_)
        batchnorm15_1_14_ = self.batchnorm15_1_14_(conv15_1_14_)
        relu15_1_14_ = self.relu15_1_14_(batchnorm15_1_14_)
        conv16_1_14_padding = self.conv16_1_14_padding(relu15_1_14_)
        conv16_1_14_ = self.conv16_1_14_(conv16_1_14_padding)
        batchnorm16_1_14_ = self.batchnorm16_1_14_(conv16_1_14_)
        relu16_1_14_ = self.relu16_1_14_(batchnorm16_1_14_)
        conv17_1_14_ = self.conv17_1_14_(relu16_1_14_)
        batchnorm17_1_14_ = self.batchnorm17_1_14_(conv17_1_14_)
        conv15_1_15_ = self.conv15_1_15_(relu13_)
        batchnorm15_1_15_ = self.batchnorm15_1_15_(conv15_1_15_)
        relu15_1_15_ = self.relu15_1_15_(batchnorm15_1_15_)
        conv16_1_15_padding = self.conv16_1_15_padding(relu15_1_15_)
        conv16_1_15_ = self.conv16_1_15_(conv16_1_15_padding)
        batchnorm16_1_15_ = self.batchnorm16_1_15_(conv16_1_15_)
        relu16_1_15_ = self.relu16_1_15_(batchnorm16_1_15_)
        conv17_1_15_ = self.conv17_1_15_(relu16_1_15_)
        batchnorm17_1_15_ = self.batchnorm17_1_15_(conv17_1_15_)
        conv15_1_16_ = self.conv15_1_16_(relu13_)
        batchnorm15_1_16_ = self.batchnorm15_1_16_(conv15_1_16_)
        relu15_1_16_ = self.relu15_1_16_(batchnorm15_1_16_)
        conv16_1_16_padding = self.conv16_1_16_padding(relu15_1_16_)
        conv16_1_16_ = self.conv16_1_16_(conv16_1_16_padding)
        batchnorm16_1_16_ = self.batchnorm16_1_16_(conv16_1_16_)
        relu16_1_16_ = self.relu16_1_16_(batchnorm16_1_16_)
        conv17_1_16_ = self.conv17_1_16_(relu16_1_16_)
        batchnorm17_1_16_ = self.batchnorm17_1_16_(conv17_1_16_)
        conv15_1_17_ = self.conv15_1_17_(relu13_)
        batchnorm15_1_17_ = self.batchnorm15_1_17_(conv15_1_17_)
        relu15_1_17_ = self.relu15_1_17_(batchnorm15_1_17_)
        conv16_1_17_padding = self.conv16_1_17_padding(relu15_1_17_)
        conv16_1_17_ = self.conv16_1_17_(conv16_1_17_padding)
        batchnorm16_1_17_ = self.batchnorm16_1_17_(conv16_1_17_)
        relu16_1_17_ = self.relu16_1_17_(batchnorm16_1_17_)
        conv17_1_17_ = self.conv17_1_17_(relu16_1_17_)
        batchnorm17_1_17_ = self.batchnorm17_1_17_(conv17_1_17_)
        conv15_1_18_ = self.conv15_1_18_(relu13_)
        batchnorm15_1_18_ = self.batchnorm15_1_18_(conv15_1_18_)
        relu15_1_18_ = self.relu15_1_18_(batchnorm15_1_18_)
        conv16_1_18_padding = self.conv16_1_18_padding(relu15_1_18_)
        conv16_1_18_ = self.conv16_1_18_(conv16_1_18_padding)
        batchnorm16_1_18_ = self.batchnorm16_1_18_(conv16_1_18_)
        relu16_1_18_ = self.relu16_1_18_(batchnorm16_1_18_)
        conv17_1_18_ = self.conv17_1_18_(relu16_1_18_)
        batchnorm17_1_18_ = self.batchnorm17_1_18_(conv17_1_18_)
        conv15_1_19_ = self.conv15_1_19_(relu13_)
        batchnorm15_1_19_ = self.batchnorm15_1_19_(conv15_1_19_)
        relu15_1_19_ = self.relu15_1_19_(batchnorm15_1_19_)
        conv16_1_19_padding = self.conv16_1_19_padding(relu15_1_19_)
        conv16_1_19_ = self.conv16_1_19_(conv16_1_19_padding)
        batchnorm16_1_19_ = self.batchnorm16_1_19_(conv16_1_19_)
        relu16_1_19_ = self.relu16_1_19_(batchnorm16_1_19_)
        conv17_1_19_ = self.conv17_1_19_(relu16_1_19_)
        batchnorm17_1_19_ = self.batchnorm17_1_19_(conv17_1_19_)
        conv15_1_20_ = self.conv15_1_20_(relu13_)
        batchnorm15_1_20_ = self.batchnorm15_1_20_(conv15_1_20_)
        relu15_1_20_ = self.relu15_1_20_(batchnorm15_1_20_)
        conv16_1_20_padding = self.conv16_1_20_padding(relu15_1_20_)
        conv16_1_20_ = self.conv16_1_20_(conv16_1_20_padding)
        batchnorm16_1_20_ = self.batchnorm16_1_20_(conv16_1_20_)
        relu16_1_20_ = self.relu16_1_20_(batchnorm16_1_20_)
        conv17_1_20_ = self.conv17_1_20_(relu16_1_20_)
        batchnorm17_1_20_ = self.batchnorm17_1_20_(conv17_1_20_)
        conv15_1_21_ = self.conv15_1_21_(relu13_)
        batchnorm15_1_21_ = self.batchnorm15_1_21_(conv15_1_21_)
        relu15_1_21_ = self.relu15_1_21_(batchnorm15_1_21_)
        conv16_1_21_padding = self.conv16_1_21_padding(relu15_1_21_)
        conv16_1_21_ = self.conv16_1_21_(conv16_1_21_padding)
        batchnorm16_1_21_ = self.batchnorm16_1_21_(conv16_1_21_)
        relu16_1_21_ = self.relu16_1_21_(batchnorm16_1_21_)
        conv17_1_21_ = self.conv17_1_21_(relu16_1_21_)
        batchnorm17_1_21_ = self.batchnorm17_1_21_(conv17_1_21_)
        conv15_1_22_ = self.conv15_1_22_(relu13_)
        batchnorm15_1_22_ = self.batchnorm15_1_22_(conv15_1_22_)
        relu15_1_22_ = self.relu15_1_22_(batchnorm15_1_22_)
        conv16_1_22_padding = self.conv16_1_22_padding(relu15_1_22_)
        conv16_1_22_ = self.conv16_1_22_(conv16_1_22_padding)
        batchnorm16_1_22_ = self.batchnorm16_1_22_(conv16_1_22_)
        relu16_1_22_ = self.relu16_1_22_(batchnorm16_1_22_)
        conv17_1_22_ = self.conv17_1_22_(relu16_1_22_)
        batchnorm17_1_22_ = self.batchnorm17_1_22_(conv17_1_22_)
        conv15_1_23_ = self.conv15_1_23_(relu13_)
        batchnorm15_1_23_ = self.batchnorm15_1_23_(conv15_1_23_)
        relu15_1_23_ = self.relu15_1_23_(batchnorm15_1_23_)
        conv16_1_23_padding = self.conv16_1_23_padding(relu15_1_23_)
        conv16_1_23_ = self.conv16_1_23_(conv16_1_23_padding)
        batchnorm16_1_23_ = self.batchnorm16_1_23_(conv16_1_23_)
        relu16_1_23_ = self.relu16_1_23_(batchnorm16_1_23_)
        conv17_1_23_ = self.conv17_1_23_(relu16_1_23_)
        batchnorm17_1_23_ = self.batchnorm17_1_23_(conv17_1_23_)
        conv15_1_24_ = self.conv15_1_24_(relu13_)
        batchnorm15_1_24_ = self.batchnorm15_1_24_(conv15_1_24_)
        relu15_1_24_ = self.relu15_1_24_(batchnorm15_1_24_)
        conv16_1_24_padding = self.conv16_1_24_padding(relu15_1_24_)
        conv16_1_24_ = self.conv16_1_24_(conv16_1_24_padding)
        batchnorm16_1_24_ = self.batchnorm16_1_24_(conv16_1_24_)
        relu16_1_24_ = self.relu16_1_24_(batchnorm16_1_24_)
        conv17_1_24_ = self.conv17_1_24_(relu16_1_24_)
        batchnorm17_1_24_ = self.batchnorm17_1_24_(conv17_1_24_)
        conv15_1_25_ = self.conv15_1_25_(relu13_)
        batchnorm15_1_25_ = self.batchnorm15_1_25_(conv15_1_25_)
        relu15_1_25_ = self.relu15_1_25_(batchnorm15_1_25_)
        conv16_1_25_padding = self.conv16_1_25_padding(relu15_1_25_)
        conv16_1_25_ = self.conv16_1_25_(conv16_1_25_padding)
        batchnorm16_1_25_ = self.batchnorm16_1_25_(conv16_1_25_)
        relu16_1_25_ = self.relu16_1_25_(batchnorm16_1_25_)
        conv17_1_25_ = self.conv17_1_25_(relu16_1_25_)
        batchnorm17_1_25_ = self.batchnorm17_1_25_(conv17_1_25_)
        conv15_1_26_ = self.conv15_1_26_(relu13_)
        batchnorm15_1_26_ = self.batchnorm15_1_26_(conv15_1_26_)
        relu15_1_26_ = self.relu15_1_26_(batchnorm15_1_26_)
        conv16_1_26_padding = self.conv16_1_26_padding(relu15_1_26_)
        conv16_1_26_ = self.conv16_1_26_(conv16_1_26_padding)
        batchnorm16_1_26_ = self.batchnorm16_1_26_(conv16_1_26_)
        relu16_1_26_ = self.relu16_1_26_(batchnorm16_1_26_)
        conv17_1_26_ = self.conv17_1_26_(relu16_1_26_)
        batchnorm17_1_26_ = self.batchnorm17_1_26_(conv17_1_26_)
        conv15_1_27_ = self.conv15_1_27_(relu13_)
        batchnorm15_1_27_ = self.batchnorm15_1_27_(conv15_1_27_)
        relu15_1_27_ = self.relu15_1_27_(batchnorm15_1_27_)
        conv16_1_27_padding = self.conv16_1_27_padding(relu15_1_27_)
        conv16_1_27_ = self.conv16_1_27_(conv16_1_27_padding)
        batchnorm16_1_27_ = self.batchnorm16_1_27_(conv16_1_27_)
        relu16_1_27_ = self.relu16_1_27_(batchnorm16_1_27_)
        conv17_1_27_ = self.conv17_1_27_(relu16_1_27_)
        batchnorm17_1_27_ = self.batchnorm17_1_27_(conv17_1_27_)
        conv15_1_28_ = self.conv15_1_28_(relu13_)
        batchnorm15_1_28_ = self.batchnorm15_1_28_(conv15_1_28_)
        relu15_1_28_ = self.relu15_1_28_(batchnorm15_1_28_)
        conv16_1_28_padding = self.conv16_1_28_padding(relu15_1_28_)
        conv16_1_28_ = self.conv16_1_28_(conv16_1_28_padding)
        batchnorm16_1_28_ = self.batchnorm16_1_28_(conv16_1_28_)
        relu16_1_28_ = self.relu16_1_28_(batchnorm16_1_28_)
        conv17_1_28_ = self.conv17_1_28_(relu16_1_28_)
        batchnorm17_1_28_ = self.batchnorm17_1_28_(conv17_1_28_)
        conv15_1_29_ = self.conv15_1_29_(relu13_)
        batchnorm15_1_29_ = self.batchnorm15_1_29_(conv15_1_29_)
        relu15_1_29_ = self.relu15_1_29_(batchnorm15_1_29_)
        conv16_1_29_padding = self.conv16_1_29_padding(relu15_1_29_)
        conv16_1_29_ = self.conv16_1_29_(conv16_1_29_padding)
        batchnorm16_1_29_ = self.batchnorm16_1_29_(conv16_1_29_)
        relu16_1_29_ = self.relu16_1_29_(batchnorm16_1_29_)
        conv17_1_29_ = self.conv17_1_29_(relu16_1_29_)
        batchnorm17_1_29_ = self.batchnorm17_1_29_(conv17_1_29_)
        conv15_1_30_ = self.conv15_1_30_(relu13_)
        batchnorm15_1_30_ = self.batchnorm15_1_30_(conv15_1_30_)
        relu15_1_30_ = self.relu15_1_30_(batchnorm15_1_30_)
        conv16_1_30_padding = self.conv16_1_30_padding(relu15_1_30_)
        conv16_1_30_ = self.conv16_1_30_(conv16_1_30_padding)
        batchnorm16_1_30_ = self.batchnorm16_1_30_(conv16_1_30_)
        relu16_1_30_ = self.relu16_1_30_(batchnorm16_1_30_)
        conv17_1_30_ = self.conv17_1_30_(relu16_1_30_)
        batchnorm17_1_30_ = self.batchnorm17_1_30_(conv17_1_30_)
        conv15_1_31_ = self.conv15_1_31_(relu13_)
        batchnorm15_1_31_ = self.batchnorm15_1_31_(conv15_1_31_)
        relu15_1_31_ = self.relu15_1_31_(batchnorm15_1_31_)
        conv16_1_31_padding = self.conv16_1_31_padding(relu15_1_31_)
        conv16_1_31_ = self.conv16_1_31_(conv16_1_31_padding)
        batchnorm16_1_31_ = self.batchnorm16_1_31_(conv16_1_31_)
        relu16_1_31_ = self.relu16_1_31_(batchnorm16_1_31_)
        conv17_1_31_ = self.conv17_1_31_(relu16_1_31_)
        batchnorm17_1_31_ = self.batchnorm17_1_31_(conv17_1_31_)
        conv15_1_32_ = self.conv15_1_32_(relu13_)
        batchnorm15_1_32_ = self.batchnorm15_1_32_(conv15_1_32_)
        relu15_1_32_ = self.relu15_1_32_(batchnorm15_1_32_)
        conv16_1_32_padding = self.conv16_1_32_padding(relu15_1_32_)
        conv16_1_32_ = self.conv16_1_32_(conv16_1_32_padding)
        batchnorm16_1_32_ = self.batchnorm16_1_32_(conv16_1_32_)
        relu16_1_32_ = self.relu16_1_32_(batchnorm16_1_32_)
        conv17_1_32_ = self.conv17_1_32_(relu16_1_32_)
        batchnorm17_1_32_ = self.batchnorm17_1_32_(conv17_1_32_)
        add18_1_ = batchnorm17_1_1_ + batchnorm17_1_2_ + batchnorm17_1_3_ + batchnorm17_1_4_ + batchnorm17_1_5_ + batchnorm17_1_6_ + batchnorm17_1_7_ + batchnorm17_1_8_ + batchnorm17_1_9_ + batchnorm17_1_10_ + batchnorm17_1_11_ + batchnorm17_1_12_ + batchnorm17_1_13_ + batchnorm17_1_14_ + batchnorm17_1_15_ + batchnorm17_1_16_ + batchnorm17_1_17_ + batchnorm17_1_18_ + batchnorm17_1_19_ + batchnorm17_1_20_ + batchnorm17_1_21_ + batchnorm17_1_22_ + batchnorm17_1_23_ + batchnorm17_1_24_ + batchnorm17_1_25_ + batchnorm17_1_26_ + batchnorm17_1_27_ + batchnorm17_1_28_ + batchnorm17_1_29_ + batchnorm17_1_30_ + batchnorm17_1_31_ + batchnorm17_1_32_
        add19_ = add18_1_ + relu13_
        relu19_ = self.relu19_(add19_)
        conv21_1_1_ = self.conv21_1_1_(relu19_)
        batchnorm21_1_1_ = self.batchnorm21_1_1_(conv21_1_1_)
        relu21_1_1_ = self.relu21_1_1_(batchnorm21_1_1_)
        conv22_1_1_padding = self.conv22_1_1_padding(relu21_1_1_)
        conv22_1_1_ = self.conv22_1_1_(conv22_1_1_padding)
        batchnorm22_1_1_ = self.batchnorm22_1_1_(conv22_1_1_)
        relu22_1_1_ = self.relu22_1_1_(batchnorm22_1_1_)
        conv23_1_1_ = self.conv23_1_1_(relu22_1_1_)
        batchnorm23_1_1_ = self.batchnorm23_1_1_(conv23_1_1_)
        conv21_1_2_ = self.conv21_1_2_(relu19_)
        batchnorm21_1_2_ = self.batchnorm21_1_2_(conv21_1_2_)
        relu21_1_2_ = self.relu21_1_2_(batchnorm21_1_2_)
        conv22_1_2_padding = self.conv22_1_2_padding(relu21_1_2_)
        conv22_1_2_ = self.conv22_1_2_(conv22_1_2_padding)
        batchnorm22_1_2_ = self.batchnorm22_1_2_(conv22_1_2_)
        relu22_1_2_ = self.relu22_1_2_(batchnorm22_1_2_)
        conv23_1_2_ = self.conv23_1_2_(relu22_1_2_)
        batchnorm23_1_2_ = self.batchnorm23_1_2_(conv23_1_2_)
        conv21_1_3_ = self.conv21_1_3_(relu19_)
        batchnorm21_1_3_ = self.batchnorm21_1_3_(conv21_1_3_)
        relu21_1_3_ = self.relu21_1_3_(batchnorm21_1_3_)
        conv22_1_3_padding = self.conv22_1_3_padding(relu21_1_3_)
        conv22_1_3_ = self.conv22_1_3_(conv22_1_3_padding)
        batchnorm22_1_3_ = self.batchnorm22_1_3_(conv22_1_3_)
        relu22_1_3_ = self.relu22_1_3_(batchnorm22_1_3_)
        conv23_1_3_ = self.conv23_1_3_(relu22_1_3_)
        batchnorm23_1_3_ = self.batchnorm23_1_3_(conv23_1_3_)
        conv21_1_4_ = self.conv21_1_4_(relu19_)
        batchnorm21_1_4_ = self.batchnorm21_1_4_(conv21_1_4_)
        relu21_1_4_ = self.relu21_1_4_(batchnorm21_1_4_)
        conv22_1_4_padding = self.conv22_1_4_padding(relu21_1_4_)
        conv22_1_4_ = self.conv22_1_4_(conv22_1_4_padding)
        batchnorm22_1_4_ = self.batchnorm22_1_4_(conv22_1_4_)
        relu22_1_4_ = self.relu22_1_4_(batchnorm22_1_4_)
        conv23_1_4_ = self.conv23_1_4_(relu22_1_4_)
        batchnorm23_1_4_ = self.batchnorm23_1_4_(conv23_1_4_)
        conv21_1_5_ = self.conv21_1_5_(relu19_)
        batchnorm21_1_5_ = self.batchnorm21_1_5_(conv21_1_5_)
        relu21_1_5_ = self.relu21_1_5_(batchnorm21_1_5_)
        conv22_1_5_padding = self.conv22_1_5_padding(relu21_1_5_)
        conv22_1_5_ = self.conv22_1_5_(conv22_1_5_padding)
        batchnorm22_1_5_ = self.batchnorm22_1_5_(conv22_1_5_)
        relu22_1_5_ = self.relu22_1_5_(batchnorm22_1_5_)
        conv23_1_5_ = self.conv23_1_5_(relu22_1_5_)
        batchnorm23_1_5_ = self.batchnorm23_1_5_(conv23_1_5_)
        conv21_1_6_ = self.conv21_1_6_(relu19_)
        batchnorm21_1_6_ = self.batchnorm21_1_6_(conv21_1_6_)
        relu21_1_6_ = self.relu21_1_6_(batchnorm21_1_6_)
        conv22_1_6_padding = self.conv22_1_6_padding(relu21_1_6_)
        conv22_1_6_ = self.conv22_1_6_(conv22_1_6_padding)
        batchnorm22_1_6_ = self.batchnorm22_1_6_(conv22_1_6_)
        relu22_1_6_ = self.relu22_1_6_(batchnorm22_1_6_)
        conv23_1_6_ = self.conv23_1_6_(relu22_1_6_)
        batchnorm23_1_6_ = self.batchnorm23_1_6_(conv23_1_6_)
        conv21_1_7_ = self.conv21_1_7_(relu19_)
        batchnorm21_1_7_ = self.batchnorm21_1_7_(conv21_1_7_)
        relu21_1_7_ = self.relu21_1_7_(batchnorm21_1_7_)
        conv22_1_7_padding = self.conv22_1_7_padding(relu21_1_7_)
        conv22_1_7_ = self.conv22_1_7_(conv22_1_7_padding)
        batchnorm22_1_7_ = self.batchnorm22_1_7_(conv22_1_7_)
        relu22_1_7_ = self.relu22_1_7_(batchnorm22_1_7_)
        conv23_1_7_ = self.conv23_1_7_(relu22_1_7_)
        batchnorm23_1_7_ = self.batchnorm23_1_7_(conv23_1_7_)
        conv21_1_8_ = self.conv21_1_8_(relu19_)
        batchnorm21_1_8_ = self.batchnorm21_1_8_(conv21_1_8_)
        relu21_1_8_ = self.relu21_1_8_(batchnorm21_1_8_)
        conv22_1_8_padding = self.conv22_1_8_padding(relu21_1_8_)
        conv22_1_8_ = self.conv22_1_8_(conv22_1_8_padding)
        batchnorm22_1_8_ = self.batchnorm22_1_8_(conv22_1_8_)
        relu22_1_8_ = self.relu22_1_8_(batchnorm22_1_8_)
        conv23_1_8_ = self.conv23_1_8_(relu22_1_8_)
        batchnorm23_1_8_ = self.batchnorm23_1_8_(conv23_1_8_)
        conv21_1_9_ = self.conv21_1_9_(relu19_)
        batchnorm21_1_9_ = self.batchnorm21_1_9_(conv21_1_9_)
        relu21_1_9_ = self.relu21_1_9_(batchnorm21_1_9_)
        conv22_1_9_padding = self.conv22_1_9_padding(relu21_1_9_)
        conv22_1_9_ = self.conv22_1_9_(conv22_1_9_padding)
        batchnorm22_1_9_ = self.batchnorm22_1_9_(conv22_1_9_)
        relu22_1_9_ = self.relu22_1_9_(batchnorm22_1_9_)
        conv23_1_9_ = self.conv23_1_9_(relu22_1_9_)
        batchnorm23_1_9_ = self.batchnorm23_1_9_(conv23_1_9_)
        conv21_1_10_ = self.conv21_1_10_(relu19_)
        batchnorm21_1_10_ = self.batchnorm21_1_10_(conv21_1_10_)
        relu21_1_10_ = self.relu21_1_10_(batchnorm21_1_10_)
        conv22_1_10_padding = self.conv22_1_10_padding(relu21_1_10_)
        conv22_1_10_ = self.conv22_1_10_(conv22_1_10_padding)
        batchnorm22_1_10_ = self.batchnorm22_1_10_(conv22_1_10_)
        relu22_1_10_ = self.relu22_1_10_(batchnorm22_1_10_)
        conv23_1_10_ = self.conv23_1_10_(relu22_1_10_)
        batchnorm23_1_10_ = self.batchnorm23_1_10_(conv23_1_10_)
        conv21_1_11_ = self.conv21_1_11_(relu19_)
        batchnorm21_1_11_ = self.batchnorm21_1_11_(conv21_1_11_)
        relu21_1_11_ = self.relu21_1_11_(batchnorm21_1_11_)
        conv22_1_11_padding = self.conv22_1_11_padding(relu21_1_11_)
        conv22_1_11_ = self.conv22_1_11_(conv22_1_11_padding)
        batchnorm22_1_11_ = self.batchnorm22_1_11_(conv22_1_11_)
        relu22_1_11_ = self.relu22_1_11_(batchnorm22_1_11_)
        conv23_1_11_ = self.conv23_1_11_(relu22_1_11_)
        batchnorm23_1_11_ = self.batchnorm23_1_11_(conv23_1_11_)
        conv21_1_12_ = self.conv21_1_12_(relu19_)
        batchnorm21_1_12_ = self.batchnorm21_1_12_(conv21_1_12_)
        relu21_1_12_ = self.relu21_1_12_(batchnorm21_1_12_)
        conv22_1_12_padding = self.conv22_1_12_padding(relu21_1_12_)
        conv22_1_12_ = self.conv22_1_12_(conv22_1_12_padding)
        batchnorm22_1_12_ = self.batchnorm22_1_12_(conv22_1_12_)
        relu22_1_12_ = self.relu22_1_12_(batchnorm22_1_12_)
        conv23_1_12_ = self.conv23_1_12_(relu22_1_12_)
        batchnorm23_1_12_ = self.batchnorm23_1_12_(conv23_1_12_)
        conv21_1_13_ = self.conv21_1_13_(relu19_)
        batchnorm21_1_13_ = self.batchnorm21_1_13_(conv21_1_13_)
        relu21_1_13_ = self.relu21_1_13_(batchnorm21_1_13_)
        conv22_1_13_padding = self.conv22_1_13_padding(relu21_1_13_)
        conv22_1_13_ = self.conv22_1_13_(conv22_1_13_padding)
        batchnorm22_1_13_ = self.batchnorm22_1_13_(conv22_1_13_)
        relu22_1_13_ = self.relu22_1_13_(batchnorm22_1_13_)
        conv23_1_13_ = self.conv23_1_13_(relu22_1_13_)
        batchnorm23_1_13_ = self.batchnorm23_1_13_(conv23_1_13_)
        conv21_1_14_ = self.conv21_1_14_(relu19_)
        batchnorm21_1_14_ = self.batchnorm21_1_14_(conv21_1_14_)
        relu21_1_14_ = self.relu21_1_14_(batchnorm21_1_14_)
        conv22_1_14_padding = self.conv22_1_14_padding(relu21_1_14_)
        conv22_1_14_ = self.conv22_1_14_(conv22_1_14_padding)
        batchnorm22_1_14_ = self.batchnorm22_1_14_(conv22_1_14_)
        relu22_1_14_ = self.relu22_1_14_(batchnorm22_1_14_)
        conv23_1_14_ = self.conv23_1_14_(relu22_1_14_)
        batchnorm23_1_14_ = self.batchnorm23_1_14_(conv23_1_14_)
        conv21_1_15_ = self.conv21_1_15_(relu19_)
        batchnorm21_1_15_ = self.batchnorm21_1_15_(conv21_1_15_)
        relu21_1_15_ = self.relu21_1_15_(batchnorm21_1_15_)
        conv22_1_15_padding = self.conv22_1_15_padding(relu21_1_15_)
        conv22_1_15_ = self.conv22_1_15_(conv22_1_15_padding)
        batchnorm22_1_15_ = self.batchnorm22_1_15_(conv22_1_15_)
        relu22_1_15_ = self.relu22_1_15_(batchnorm22_1_15_)
        conv23_1_15_ = self.conv23_1_15_(relu22_1_15_)
        batchnorm23_1_15_ = self.batchnorm23_1_15_(conv23_1_15_)
        conv21_1_16_ = self.conv21_1_16_(relu19_)
        batchnorm21_1_16_ = self.batchnorm21_1_16_(conv21_1_16_)
        relu21_1_16_ = self.relu21_1_16_(batchnorm21_1_16_)
        conv22_1_16_padding = self.conv22_1_16_padding(relu21_1_16_)
        conv22_1_16_ = self.conv22_1_16_(conv22_1_16_padding)
        batchnorm22_1_16_ = self.batchnorm22_1_16_(conv22_1_16_)
        relu22_1_16_ = self.relu22_1_16_(batchnorm22_1_16_)
        conv23_1_16_ = self.conv23_1_16_(relu22_1_16_)
        batchnorm23_1_16_ = self.batchnorm23_1_16_(conv23_1_16_)
        conv21_1_17_ = self.conv21_1_17_(relu19_)
        batchnorm21_1_17_ = self.batchnorm21_1_17_(conv21_1_17_)
        relu21_1_17_ = self.relu21_1_17_(batchnorm21_1_17_)
        conv22_1_17_padding = self.conv22_1_17_padding(relu21_1_17_)
        conv22_1_17_ = self.conv22_1_17_(conv22_1_17_padding)
        batchnorm22_1_17_ = self.batchnorm22_1_17_(conv22_1_17_)
        relu22_1_17_ = self.relu22_1_17_(batchnorm22_1_17_)
        conv23_1_17_ = self.conv23_1_17_(relu22_1_17_)
        batchnorm23_1_17_ = self.batchnorm23_1_17_(conv23_1_17_)
        conv21_1_18_ = self.conv21_1_18_(relu19_)
        batchnorm21_1_18_ = self.batchnorm21_1_18_(conv21_1_18_)
        relu21_1_18_ = self.relu21_1_18_(batchnorm21_1_18_)
        conv22_1_18_padding = self.conv22_1_18_padding(relu21_1_18_)
        conv22_1_18_ = self.conv22_1_18_(conv22_1_18_padding)
        batchnorm22_1_18_ = self.batchnorm22_1_18_(conv22_1_18_)
        relu22_1_18_ = self.relu22_1_18_(batchnorm22_1_18_)
        conv23_1_18_ = self.conv23_1_18_(relu22_1_18_)
        batchnorm23_1_18_ = self.batchnorm23_1_18_(conv23_1_18_)
        conv21_1_19_ = self.conv21_1_19_(relu19_)
        batchnorm21_1_19_ = self.batchnorm21_1_19_(conv21_1_19_)
        relu21_1_19_ = self.relu21_1_19_(batchnorm21_1_19_)
        conv22_1_19_padding = self.conv22_1_19_padding(relu21_1_19_)
        conv22_1_19_ = self.conv22_1_19_(conv22_1_19_padding)
        batchnorm22_1_19_ = self.batchnorm22_1_19_(conv22_1_19_)
        relu22_1_19_ = self.relu22_1_19_(batchnorm22_1_19_)
        conv23_1_19_ = self.conv23_1_19_(relu22_1_19_)
        batchnorm23_1_19_ = self.batchnorm23_1_19_(conv23_1_19_)
        conv21_1_20_ = self.conv21_1_20_(relu19_)
        batchnorm21_1_20_ = self.batchnorm21_1_20_(conv21_1_20_)
        relu21_1_20_ = self.relu21_1_20_(batchnorm21_1_20_)
        conv22_1_20_padding = self.conv22_1_20_padding(relu21_1_20_)
        conv22_1_20_ = self.conv22_1_20_(conv22_1_20_padding)
        batchnorm22_1_20_ = self.batchnorm22_1_20_(conv22_1_20_)
        relu22_1_20_ = self.relu22_1_20_(batchnorm22_1_20_)
        conv23_1_20_ = self.conv23_1_20_(relu22_1_20_)
        batchnorm23_1_20_ = self.batchnorm23_1_20_(conv23_1_20_)
        conv21_1_21_ = self.conv21_1_21_(relu19_)
        batchnorm21_1_21_ = self.batchnorm21_1_21_(conv21_1_21_)
        relu21_1_21_ = self.relu21_1_21_(batchnorm21_1_21_)
        conv22_1_21_padding = self.conv22_1_21_padding(relu21_1_21_)
        conv22_1_21_ = self.conv22_1_21_(conv22_1_21_padding)
        batchnorm22_1_21_ = self.batchnorm22_1_21_(conv22_1_21_)
        relu22_1_21_ = self.relu22_1_21_(batchnorm22_1_21_)
        conv23_1_21_ = self.conv23_1_21_(relu22_1_21_)
        batchnorm23_1_21_ = self.batchnorm23_1_21_(conv23_1_21_)
        conv21_1_22_ = self.conv21_1_22_(relu19_)
        batchnorm21_1_22_ = self.batchnorm21_1_22_(conv21_1_22_)
        relu21_1_22_ = self.relu21_1_22_(batchnorm21_1_22_)
        conv22_1_22_padding = self.conv22_1_22_padding(relu21_1_22_)
        conv22_1_22_ = self.conv22_1_22_(conv22_1_22_padding)
        batchnorm22_1_22_ = self.batchnorm22_1_22_(conv22_1_22_)
        relu22_1_22_ = self.relu22_1_22_(batchnorm22_1_22_)
        conv23_1_22_ = self.conv23_1_22_(relu22_1_22_)
        batchnorm23_1_22_ = self.batchnorm23_1_22_(conv23_1_22_)
        conv21_1_23_ = self.conv21_1_23_(relu19_)
        batchnorm21_1_23_ = self.batchnorm21_1_23_(conv21_1_23_)
        relu21_1_23_ = self.relu21_1_23_(batchnorm21_1_23_)
        conv22_1_23_padding = self.conv22_1_23_padding(relu21_1_23_)
        conv22_1_23_ = self.conv22_1_23_(conv22_1_23_padding)
        batchnorm22_1_23_ = self.batchnorm22_1_23_(conv22_1_23_)
        relu22_1_23_ = self.relu22_1_23_(batchnorm22_1_23_)
        conv23_1_23_ = self.conv23_1_23_(relu22_1_23_)
        batchnorm23_1_23_ = self.batchnorm23_1_23_(conv23_1_23_)
        conv21_1_24_ = self.conv21_1_24_(relu19_)
        batchnorm21_1_24_ = self.batchnorm21_1_24_(conv21_1_24_)
        relu21_1_24_ = self.relu21_1_24_(batchnorm21_1_24_)
        conv22_1_24_padding = self.conv22_1_24_padding(relu21_1_24_)
        conv22_1_24_ = self.conv22_1_24_(conv22_1_24_padding)
        batchnorm22_1_24_ = self.batchnorm22_1_24_(conv22_1_24_)
        relu22_1_24_ = self.relu22_1_24_(batchnorm22_1_24_)
        conv23_1_24_ = self.conv23_1_24_(relu22_1_24_)
        batchnorm23_1_24_ = self.batchnorm23_1_24_(conv23_1_24_)
        conv21_1_25_ = self.conv21_1_25_(relu19_)
        batchnorm21_1_25_ = self.batchnorm21_1_25_(conv21_1_25_)
        relu21_1_25_ = self.relu21_1_25_(batchnorm21_1_25_)
        conv22_1_25_padding = self.conv22_1_25_padding(relu21_1_25_)
        conv22_1_25_ = self.conv22_1_25_(conv22_1_25_padding)
        batchnorm22_1_25_ = self.batchnorm22_1_25_(conv22_1_25_)
        relu22_1_25_ = self.relu22_1_25_(batchnorm22_1_25_)
        conv23_1_25_ = self.conv23_1_25_(relu22_1_25_)
        batchnorm23_1_25_ = self.batchnorm23_1_25_(conv23_1_25_)
        conv21_1_26_ = self.conv21_1_26_(relu19_)
        batchnorm21_1_26_ = self.batchnorm21_1_26_(conv21_1_26_)
        relu21_1_26_ = self.relu21_1_26_(batchnorm21_1_26_)
        conv22_1_26_padding = self.conv22_1_26_padding(relu21_1_26_)
        conv22_1_26_ = self.conv22_1_26_(conv22_1_26_padding)
        batchnorm22_1_26_ = self.batchnorm22_1_26_(conv22_1_26_)
        relu22_1_26_ = self.relu22_1_26_(batchnorm22_1_26_)
        conv23_1_26_ = self.conv23_1_26_(relu22_1_26_)
        batchnorm23_1_26_ = self.batchnorm23_1_26_(conv23_1_26_)
        conv21_1_27_ = self.conv21_1_27_(relu19_)
        batchnorm21_1_27_ = self.batchnorm21_1_27_(conv21_1_27_)
        relu21_1_27_ = self.relu21_1_27_(batchnorm21_1_27_)
        conv22_1_27_padding = self.conv22_1_27_padding(relu21_1_27_)
        conv22_1_27_ = self.conv22_1_27_(conv22_1_27_padding)
        batchnorm22_1_27_ = self.batchnorm22_1_27_(conv22_1_27_)
        relu22_1_27_ = self.relu22_1_27_(batchnorm22_1_27_)
        conv23_1_27_ = self.conv23_1_27_(relu22_1_27_)
        batchnorm23_1_27_ = self.batchnorm23_1_27_(conv23_1_27_)
        conv21_1_28_ = self.conv21_1_28_(relu19_)
        batchnorm21_1_28_ = self.batchnorm21_1_28_(conv21_1_28_)
        relu21_1_28_ = self.relu21_1_28_(batchnorm21_1_28_)
        conv22_1_28_padding = self.conv22_1_28_padding(relu21_1_28_)
        conv22_1_28_ = self.conv22_1_28_(conv22_1_28_padding)
        batchnorm22_1_28_ = self.batchnorm22_1_28_(conv22_1_28_)
        relu22_1_28_ = self.relu22_1_28_(batchnorm22_1_28_)
        conv23_1_28_ = self.conv23_1_28_(relu22_1_28_)
        batchnorm23_1_28_ = self.batchnorm23_1_28_(conv23_1_28_)
        conv21_1_29_ = self.conv21_1_29_(relu19_)
        batchnorm21_1_29_ = self.batchnorm21_1_29_(conv21_1_29_)
        relu21_1_29_ = self.relu21_1_29_(batchnorm21_1_29_)
        conv22_1_29_padding = self.conv22_1_29_padding(relu21_1_29_)
        conv22_1_29_ = self.conv22_1_29_(conv22_1_29_padding)
        batchnorm22_1_29_ = self.batchnorm22_1_29_(conv22_1_29_)
        relu22_1_29_ = self.relu22_1_29_(batchnorm22_1_29_)
        conv23_1_29_ = self.conv23_1_29_(relu22_1_29_)
        batchnorm23_1_29_ = self.batchnorm23_1_29_(conv23_1_29_)
        conv21_1_30_ = self.conv21_1_30_(relu19_)
        batchnorm21_1_30_ = self.batchnorm21_1_30_(conv21_1_30_)
        relu21_1_30_ = self.relu21_1_30_(batchnorm21_1_30_)
        conv22_1_30_padding = self.conv22_1_30_padding(relu21_1_30_)
        conv22_1_30_ = self.conv22_1_30_(conv22_1_30_padding)
        batchnorm22_1_30_ = self.batchnorm22_1_30_(conv22_1_30_)
        relu22_1_30_ = self.relu22_1_30_(batchnorm22_1_30_)
        conv23_1_30_ = self.conv23_1_30_(relu22_1_30_)
        batchnorm23_1_30_ = self.batchnorm23_1_30_(conv23_1_30_)
        conv21_1_31_ = self.conv21_1_31_(relu19_)
        batchnorm21_1_31_ = self.batchnorm21_1_31_(conv21_1_31_)
        relu21_1_31_ = self.relu21_1_31_(batchnorm21_1_31_)
        conv22_1_31_padding = self.conv22_1_31_padding(relu21_1_31_)
        conv22_1_31_ = self.conv22_1_31_(conv22_1_31_padding)
        batchnorm22_1_31_ = self.batchnorm22_1_31_(conv22_1_31_)
        relu22_1_31_ = self.relu22_1_31_(batchnorm22_1_31_)
        conv23_1_31_ = self.conv23_1_31_(relu22_1_31_)
        batchnorm23_1_31_ = self.batchnorm23_1_31_(conv23_1_31_)
        conv21_1_32_ = self.conv21_1_32_(relu19_)
        batchnorm21_1_32_ = self.batchnorm21_1_32_(conv21_1_32_)
        relu21_1_32_ = self.relu21_1_32_(batchnorm21_1_32_)
        conv22_1_32_padding = self.conv22_1_32_padding(relu21_1_32_)
        conv22_1_32_ = self.conv22_1_32_(conv22_1_32_padding)
        batchnorm22_1_32_ = self.batchnorm22_1_32_(conv22_1_32_)
        relu22_1_32_ = self.relu22_1_32_(batchnorm22_1_32_)
        conv23_1_32_ = self.conv23_1_32_(relu22_1_32_)
        batchnorm23_1_32_ = self.batchnorm23_1_32_(conv23_1_32_)
        add24_1_ = batchnorm23_1_1_ + batchnorm23_1_2_ + batchnorm23_1_3_ + batchnorm23_1_4_ + batchnorm23_1_5_ + batchnorm23_1_6_ + batchnorm23_1_7_ + batchnorm23_1_8_ + batchnorm23_1_9_ + batchnorm23_1_10_ + batchnorm23_1_11_ + batchnorm23_1_12_ + batchnorm23_1_13_ + batchnorm23_1_14_ + batchnorm23_1_15_ + batchnorm23_1_16_ + batchnorm23_1_17_ + batchnorm23_1_18_ + batchnorm23_1_19_ + batchnorm23_1_20_ + batchnorm23_1_21_ + batchnorm23_1_22_ + batchnorm23_1_23_ + batchnorm23_1_24_ + batchnorm23_1_25_ + batchnorm23_1_26_ + batchnorm23_1_27_ + batchnorm23_1_28_ + batchnorm23_1_29_ + batchnorm23_1_30_ + batchnorm23_1_31_ + batchnorm23_1_32_
        conv20_2_ = self.conv20_2_(relu19_)
        batchnorm20_2_ = self.batchnorm20_2_(conv20_2_)
        add25_ = add24_1_ + batchnorm20_2_
        relu25_ = self.relu25_(add25_)
        conv27_1_1_ = self.conv27_1_1_(relu25_)
        batchnorm27_1_1_ = self.batchnorm27_1_1_(conv27_1_1_)
        relu27_1_1_ = self.relu27_1_1_(batchnorm27_1_1_)
        conv28_1_1_padding = self.conv28_1_1_padding(relu27_1_1_)
        conv28_1_1_ = self.conv28_1_1_(conv28_1_1_padding)
        batchnorm28_1_1_ = self.batchnorm28_1_1_(conv28_1_1_)
        relu28_1_1_ = self.relu28_1_1_(batchnorm28_1_1_)
        conv29_1_1_ = self.conv29_1_1_(relu28_1_1_)
        batchnorm29_1_1_ = self.batchnorm29_1_1_(conv29_1_1_)
        conv27_1_2_ = self.conv27_1_2_(relu25_)
        batchnorm27_1_2_ = self.batchnorm27_1_2_(conv27_1_2_)
        relu27_1_2_ = self.relu27_1_2_(batchnorm27_1_2_)
        conv28_1_2_padding = self.conv28_1_2_padding(relu27_1_2_)
        conv28_1_2_ = self.conv28_1_2_(conv28_1_2_padding)
        batchnorm28_1_2_ = self.batchnorm28_1_2_(conv28_1_2_)
        relu28_1_2_ = self.relu28_1_2_(batchnorm28_1_2_)
        conv29_1_2_ = self.conv29_1_2_(relu28_1_2_)
        batchnorm29_1_2_ = self.batchnorm29_1_2_(conv29_1_2_)
        conv27_1_3_ = self.conv27_1_3_(relu25_)
        batchnorm27_1_3_ = self.batchnorm27_1_3_(conv27_1_3_)
        relu27_1_3_ = self.relu27_1_3_(batchnorm27_1_3_)
        conv28_1_3_padding = self.conv28_1_3_padding(relu27_1_3_)
        conv28_1_3_ = self.conv28_1_3_(conv28_1_3_padding)
        batchnorm28_1_3_ = self.batchnorm28_1_3_(conv28_1_3_)
        relu28_1_3_ = self.relu28_1_3_(batchnorm28_1_3_)
        conv29_1_3_ = self.conv29_1_3_(relu28_1_3_)
        batchnorm29_1_3_ = self.batchnorm29_1_3_(conv29_1_3_)
        conv27_1_4_ = self.conv27_1_4_(relu25_)
        batchnorm27_1_4_ = self.batchnorm27_1_4_(conv27_1_4_)
        relu27_1_4_ = self.relu27_1_4_(batchnorm27_1_4_)
        conv28_1_4_padding = self.conv28_1_4_padding(relu27_1_4_)
        conv28_1_4_ = self.conv28_1_4_(conv28_1_4_padding)
        batchnorm28_1_4_ = self.batchnorm28_1_4_(conv28_1_4_)
        relu28_1_4_ = self.relu28_1_4_(batchnorm28_1_4_)
        conv29_1_4_ = self.conv29_1_4_(relu28_1_4_)
        batchnorm29_1_4_ = self.batchnorm29_1_4_(conv29_1_4_)
        conv27_1_5_ = self.conv27_1_5_(relu25_)
        batchnorm27_1_5_ = self.batchnorm27_1_5_(conv27_1_5_)
        relu27_1_5_ = self.relu27_1_5_(batchnorm27_1_5_)
        conv28_1_5_padding = self.conv28_1_5_padding(relu27_1_5_)
        conv28_1_5_ = self.conv28_1_5_(conv28_1_5_padding)
        batchnorm28_1_5_ = self.batchnorm28_1_5_(conv28_1_5_)
        relu28_1_5_ = self.relu28_1_5_(batchnorm28_1_5_)
        conv29_1_5_ = self.conv29_1_5_(relu28_1_5_)
        batchnorm29_1_5_ = self.batchnorm29_1_5_(conv29_1_5_)
        conv27_1_6_ = self.conv27_1_6_(relu25_)
        batchnorm27_1_6_ = self.batchnorm27_1_6_(conv27_1_6_)
        relu27_1_6_ = self.relu27_1_6_(batchnorm27_1_6_)
        conv28_1_6_padding = self.conv28_1_6_padding(relu27_1_6_)
        conv28_1_6_ = self.conv28_1_6_(conv28_1_6_padding)
        batchnorm28_1_6_ = self.batchnorm28_1_6_(conv28_1_6_)
        relu28_1_6_ = self.relu28_1_6_(batchnorm28_1_6_)
        conv29_1_6_ = self.conv29_1_6_(relu28_1_6_)
        batchnorm29_1_6_ = self.batchnorm29_1_6_(conv29_1_6_)
        conv27_1_7_ = self.conv27_1_7_(relu25_)
        batchnorm27_1_7_ = self.batchnorm27_1_7_(conv27_1_7_)
        relu27_1_7_ = self.relu27_1_7_(batchnorm27_1_7_)
        conv28_1_7_padding = self.conv28_1_7_padding(relu27_1_7_)
        conv28_1_7_ = self.conv28_1_7_(conv28_1_7_padding)
        batchnorm28_1_7_ = self.batchnorm28_1_7_(conv28_1_7_)
        relu28_1_7_ = self.relu28_1_7_(batchnorm28_1_7_)
        conv29_1_7_ = self.conv29_1_7_(relu28_1_7_)
        batchnorm29_1_7_ = self.batchnorm29_1_7_(conv29_1_7_)
        conv27_1_8_ = self.conv27_1_8_(relu25_)
        batchnorm27_1_8_ = self.batchnorm27_1_8_(conv27_1_8_)
        relu27_1_8_ = self.relu27_1_8_(batchnorm27_1_8_)
        conv28_1_8_padding = self.conv28_1_8_padding(relu27_1_8_)
        conv28_1_8_ = self.conv28_1_8_(conv28_1_8_padding)
        batchnorm28_1_8_ = self.batchnorm28_1_8_(conv28_1_8_)
        relu28_1_8_ = self.relu28_1_8_(batchnorm28_1_8_)
        conv29_1_8_ = self.conv29_1_8_(relu28_1_8_)
        batchnorm29_1_8_ = self.batchnorm29_1_8_(conv29_1_8_)
        conv27_1_9_ = self.conv27_1_9_(relu25_)
        batchnorm27_1_9_ = self.batchnorm27_1_9_(conv27_1_9_)
        relu27_1_9_ = self.relu27_1_9_(batchnorm27_1_9_)
        conv28_1_9_padding = self.conv28_1_9_padding(relu27_1_9_)
        conv28_1_9_ = self.conv28_1_9_(conv28_1_9_padding)
        batchnorm28_1_9_ = self.batchnorm28_1_9_(conv28_1_9_)
        relu28_1_9_ = self.relu28_1_9_(batchnorm28_1_9_)
        conv29_1_9_ = self.conv29_1_9_(relu28_1_9_)
        batchnorm29_1_9_ = self.batchnorm29_1_9_(conv29_1_9_)
        conv27_1_10_ = self.conv27_1_10_(relu25_)
        batchnorm27_1_10_ = self.batchnorm27_1_10_(conv27_1_10_)
        relu27_1_10_ = self.relu27_1_10_(batchnorm27_1_10_)
        conv28_1_10_padding = self.conv28_1_10_padding(relu27_1_10_)
        conv28_1_10_ = self.conv28_1_10_(conv28_1_10_padding)
        batchnorm28_1_10_ = self.batchnorm28_1_10_(conv28_1_10_)
        relu28_1_10_ = self.relu28_1_10_(batchnorm28_1_10_)
        conv29_1_10_ = self.conv29_1_10_(relu28_1_10_)
        batchnorm29_1_10_ = self.batchnorm29_1_10_(conv29_1_10_)
        conv27_1_11_ = self.conv27_1_11_(relu25_)
        batchnorm27_1_11_ = self.batchnorm27_1_11_(conv27_1_11_)
        relu27_1_11_ = self.relu27_1_11_(batchnorm27_1_11_)
        conv28_1_11_padding = self.conv28_1_11_padding(relu27_1_11_)
        conv28_1_11_ = self.conv28_1_11_(conv28_1_11_padding)
        batchnorm28_1_11_ = self.batchnorm28_1_11_(conv28_1_11_)
        relu28_1_11_ = self.relu28_1_11_(batchnorm28_1_11_)
        conv29_1_11_ = self.conv29_1_11_(relu28_1_11_)
        batchnorm29_1_11_ = self.batchnorm29_1_11_(conv29_1_11_)
        conv27_1_12_ = self.conv27_1_12_(relu25_)
        batchnorm27_1_12_ = self.batchnorm27_1_12_(conv27_1_12_)
        relu27_1_12_ = self.relu27_1_12_(batchnorm27_1_12_)
        conv28_1_12_padding = self.conv28_1_12_padding(relu27_1_12_)
        conv28_1_12_ = self.conv28_1_12_(conv28_1_12_padding)
        batchnorm28_1_12_ = self.batchnorm28_1_12_(conv28_1_12_)
        relu28_1_12_ = self.relu28_1_12_(batchnorm28_1_12_)
        conv29_1_12_ = self.conv29_1_12_(relu28_1_12_)
        batchnorm29_1_12_ = self.batchnorm29_1_12_(conv29_1_12_)
        conv27_1_13_ = self.conv27_1_13_(relu25_)
        batchnorm27_1_13_ = self.batchnorm27_1_13_(conv27_1_13_)
        relu27_1_13_ = self.relu27_1_13_(batchnorm27_1_13_)
        conv28_1_13_padding = self.conv28_1_13_padding(relu27_1_13_)
        conv28_1_13_ = self.conv28_1_13_(conv28_1_13_padding)
        batchnorm28_1_13_ = self.batchnorm28_1_13_(conv28_1_13_)
        relu28_1_13_ = self.relu28_1_13_(batchnorm28_1_13_)
        conv29_1_13_ = self.conv29_1_13_(relu28_1_13_)
        batchnorm29_1_13_ = self.batchnorm29_1_13_(conv29_1_13_)
        conv27_1_14_ = self.conv27_1_14_(relu25_)
        batchnorm27_1_14_ = self.batchnorm27_1_14_(conv27_1_14_)
        relu27_1_14_ = self.relu27_1_14_(batchnorm27_1_14_)
        conv28_1_14_padding = self.conv28_1_14_padding(relu27_1_14_)
        conv28_1_14_ = self.conv28_1_14_(conv28_1_14_padding)
        batchnorm28_1_14_ = self.batchnorm28_1_14_(conv28_1_14_)
        relu28_1_14_ = self.relu28_1_14_(batchnorm28_1_14_)
        conv29_1_14_ = self.conv29_1_14_(relu28_1_14_)
        batchnorm29_1_14_ = self.batchnorm29_1_14_(conv29_1_14_)
        conv27_1_15_ = self.conv27_1_15_(relu25_)
        batchnorm27_1_15_ = self.batchnorm27_1_15_(conv27_1_15_)
        relu27_1_15_ = self.relu27_1_15_(batchnorm27_1_15_)
        conv28_1_15_padding = self.conv28_1_15_padding(relu27_1_15_)
        conv28_1_15_ = self.conv28_1_15_(conv28_1_15_padding)
        batchnorm28_1_15_ = self.batchnorm28_1_15_(conv28_1_15_)
        relu28_1_15_ = self.relu28_1_15_(batchnorm28_1_15_)
        conv29_1_15_ = self.conv29_1_15_(relu28_1_15_)
        batchnorm29_1_15_ = self.batchnorm29_1_15_(conv29_1_15_)
        conv27_1_16_ = self.conv27_1_16_(relu25_)
        batchnorm27_1_16_ = self.batchnorm27_1_16_(conv27_1_16_)
        relu27_1_16_ = self.relu27_1_16_(batchnorm27_1_16_)
        conv28_1_16_padding = self.conv28_1_16_padding(relu27_1_16_)
        conv28_1_16_ = self.conv28_1_16_(conv28_1_16_padding)
        batchnorm28_1_16_ = self.batchnorm28_1_16_(conv28_1_16_)
        relu28_1_16_ = self.relu28_1_16_(batchnorm28_1_16_)
        conv29_1_16_ = self.conv29_1_16_(relu28_1_16_)
        batchnorm29_1_16_ = self.batchnorm29_1_16_(conv29_1_16_)
        conv27_1_17_ = self.conv27_1_17_(relu25_)
        batchnorm27_1_17_ = self.batchnorm27_1_17_(conv27_1_17_)
        relu27_1_17_ = self.relu27_1_17_(batchnorm27_1_17_)
        conv28_1_17_padding = self.conv28_1_17_padding(relu27_1_17_)
        conv28_1_17_ = self.conv28_1_17_(conv28_1_17_padding)
        batchnorm28_1_17_ = self.batchnorm28_1_17_(conv28_1_17_)
        relu28_1_17_ = self.relu28_1_17_(batchnorm28_1_17_)
        conv29_1_17_ = self.conv29_1_17_(relu28_1_17_)
        batchnorm29_1_17_ = self.batchnorm29_1_17_(conv29_1_17_)
        conv27_1_18_ = self.conv27_1_18_(relu25_)
        batchnorm27_1_18_ = self.batchnorm27_1_18_(conv27_1_18_)
        relu27_1_18_ = self.relu27_1_18_(batchnorm27_1_18_)
        conv28_1_18_padding = self.conv28_1_18_padding(relu27_1_18_)
        conv28_1_18_ = self.conv28_1_18_(conv28_1_18_padding)
        batchnorm28_1_18_ = self.batchnorm28_1_18_(conv28_1_18_)
        relu28_1_18_ = self.relu28_1_18_(batchnorm28_1_18_)
        conv29_1_18_ = self.conv29_1_18_(relu28_1_18_)
        batchnorm29_1_18_ = self.batchnorm29_1_18_(conv29_1_18_)
        conv27_1_19_ = self.conv27_1_19_(relu25_)
        batchnorm27_1_19_ = self.batchnorm27_1_19_(conv27_1_19_)
        relu27_1_19_ = self.relu27_1_19_(batchnorm27_1_19_)
        conv28_1_19_padding = self.conv28_1_19_padding(relu27_1_19_)
        conv28_1_19_ = self.conv28_1_19_(conv28_1_19_padding)
        batchnorm28_1_19_ = self.batchnorm28_1_19_(conv28_1_19_)
        relu28_1_19_ = self.relu28_1_19_(batchnorm28_1_19_)
        conv29_1_19_ = self.conv29_1_19_(relu28_1_19_)
        batchnorm29_1_19_ = self.batchnorm29_1_19_(conv29_1_19_)
        conv27_1_20_ = self.conv27_1_20_(relu25_)
        batchnorm27_1_20_ = self.batchnorm27_1_20_(conv27_1_20_)
        relu27_1_20_ = self.relu27_1_20_(batchnorm27_1_20_)
        conv28_1_20_padding = self.conv28_1_20_padding(relu27_1_20_)
        conv28_1_20_ = self.conv28_1_20_(conv28_1_20_padding)
        batchnorm28_1_20_ = self.batchnorm28_1_20_(conv28_1_20_)
        relu28_1_20_ = self.relu28_1_20_(batchnorm28_1_20_)
        conv29_1_20_ = self.conv29_1_20_(relu28_1_20_)
        batchnorm29_1_20_ = self.batchnorm29_1_20_(conv29_1_20_)
        conv27_1_21_ = self.conv27_1_21_(relu25_)
        batchnorm27_1_21_ = self.batchnorm27_1_21_(conv27_1_21_)
        relu27_1_21_ = self.relu27_1_21_(batchnorm27_1_21_)
        conv28_1_21_padding = self.conv28_1_21_padding(relu27_1_21_)
        conv28_1_21_ = self.conv28_1_21_(conv28_1_21_padding)
        batchnorm28_1_21_ = self.batchnorm28_1_21_(conv28_1_21_)
        relu28_1_21_ = self.relu28_1_21_(batchnorm28_1_21_)
        conv29_1_21_ = self.conv29_1_21_(relu28_1_21_)
        batchnorm29_1_21_ = self.batchnorm29_1_21_(conv29_1_21_)
        conv27_1_22_ = self.conv27_1_22_(relu25_)
        batchnorm27_1_22_ = self.batchnorm27_1_22_(conv27_1_22_)
        relu27_1_22_ = self.relu27_1_22_(batchnorm27_1_22_)
        conv28_1_22_padding = self.conv28_1_22_padding(relu27_1_22_)
        conv28_1_22_ = self.conv28_1_22_(conv28_1_22_padding)
        batchnorm28_1_22_ = self.batchnorm28_1_22_(conv28_1_22_)
        relu28_1_22_ = self.relu28_1_22_(batchnorm28_1_22_)
        conv29_1_22_ = self.conv29_1_22_(relu28_1_22_)
        batchnorm29_1_22_ = self.batchnorm29_1_22_(conv29_1_22_)
        conv27_1_23_ = self.conv27_1_23_(relu25_)
        batchnorm27_1_23_ = self.batchnorm27_1_23_(conv27_1_23_)
        relu27_1_23_ = self.relu27_1_23_(batchnorm27_1_23_)
        conv28_1_23_padding = self.conv28_1_23_padding(relu27_1_23_)
        conv28_1_23_ = self.conv28_1_23_(conv28_1_23_padding)
        batchnorm28_1_23_ = self.batchnorm28_1_23_(conv28_1_23_)
        relu28_1_23_ = self.relu28_1_23_(batchnorm28_1_23_)
        conv29_1_23_ = self.conv29_1_23_(relu28_1_23_)
        batchnorm29_1_23_ = self.batchnorm29_1_23_(conv29_1_23_)
        conv27_1_24_ = self.conv27_1_24_(relu25_)
        batchnorm27_1_24_ = self.batchnorm27_1_24_(conv27_1_24_)
        relu27_1_24_ = self.relu27_1_24_(batchnorm27_1_24_)
        conv28_1_24_padding = self.conv28_1_24_padding(relu27_1_24_)
        conv28_1_24_ = self.conv28_1_24_(conv28_1_24_padding)
        batchnorm28_1_24_ = self.batchnorm28_1_24_(conv28_1_24_)
        relu28_1_24_ = self.relu28_1_24_(batchnorm28_1_24_)
        conv29_1_24_ = self.conv29_1_24_(relu28_1_24_)
        batchnorm29_1_24_ = self.batchnorm29_1_24_(conv29_1_24_)
        conv27_1_25_ = self.conv27_1_25_(relu25_)
        batchnorm27_1_25_ = self.batchnorm27_1_25_(conv27_1_25_)
        relu27_1_25_ = self.relu27_1_25_(batchnorm27_1_25_)
        conv28_1_25_padding = self.conv28_1_25_padding(relu27_1_25_)
        conv28_1_25_ = self.conv28_1_25_(conv28_1_25_padding)
        batchnorm28_1_25_ = self.batchnorm28_1_25_(conv28_1_25_)
        relu28_1_25_ = self.relu28_1_25_(batchnorm28_1_25_)
        conv29_1_25_ = self.conv29_1_25_(relu28_1_25_)
        batchnorm29_1_25_ = self.batchnorm29_1_25_(conv29_1_25_)
        conv27_1_26_ = self.conv27_1_26_(relu25_)
        batchnorm27_1_26_ = self.batchnorm27_1_26_(conv27_1_26_)
        relu27_1_26_ = self.relu27_1_26_(batchnorm27_1_26_)
        conv28_1_26_padding = self.conv28_1_26_padding(relu27_1_26_)
        conv28_1_26_ = self.conv28_1_26_(conv28_1_26_padding)
        batchnorm28_1_26_ = self.batchnorm28_1_26_(conv28_1_26_)
        relu28_1_26_ = self.relu28_1_26_(batchnorm28_1_26_)
        conv29_1_26_ = self.conv29_1_26_(relu28_1_26_)
        batchnorm29_1_26_ = self.batchnorm29_1_26_(conv29_1_26_)
        conv27_1_27_ = self.conv27_1_27_(relu25_)
        batchnorm27_1_27_ = self.batchnorm27_1_27_(conv27_1_27_)
        relu27_1_27_ = self.relu27_1_27_(batchnorm27_1_27_)
        conv28_1_27_padding = self.conv28_1_27_padding(relu27_1_27_)
        conv28_1_27_ = self.conv28_1_27_(conv28_1_27_padding)
        batchnorm28_1_27_ = self.batchnorm28_1_27_(conv28_1_27_)
        relu28_1_27_ = self.relu28_1_27_(batchnorm28_1_27_)
        conv29_1_27_ = self.conv29_1_27_(relu28_1_27_)
        batchnorm29_1_27_ = self.batchnorm29_1_27_(conv29_1_27_)
        conv27_1_28_ = self.conv27_1_28_(relu25_)
        batchnorm27_1_28_ = self.batchnorm27_1_28_(conv27_1_28_)
        relu27_1_28_ = self.relu27_1_28_(batchnorm27_1_28_)
        conv28_1_28_padding = self.conv28_1_28_padding(relu27_1_28_)
        conv28_1_28_ = self.conv28_1_28_(conv28_1_28_padding)
        batchnorm28_1_28_ = self.batchnorm28_1_28_(conv28_1_28_)
        relu28_1_28_ = self.relu28_1_28_(batchnorm28_1_28_)
        conv29_1_28_ = self.conv29_1_28_(relu28_1_28_)
        batchnorm29_1_28_ = self.batchnorm29_1_28_(conv29_1_28_)
        conv27_1_29_ = self.conv27_1_29_(relu25_)
        batchnorm27_1_29_ = self.batchnorm27_1_29_(conv27_1_29_)
        relu27_1_29_ = self.relu27_1_29_(batchnorm27_1_29_)
        conv28_1_29_padding = self.conv28_1_29_padding(relu27_1_29_)
        conv28_1_29_ = self.conv28_1_29_(conv28_1_29_padding)
        batchnorm28_1_29_ = self.batchnorm28_1_29_(conv28_1_29_)
        relu28_1_29_ = self.relu28_1_29_(batchnorm28_1_29_)
        conv29_1_29_ = self.conv29_1_29_(relu28_1_29_)
        batchnorm29_1_29_ = self.batchnorm29_1_29_(conv29_1_29_)
        conv27_1_30_ = self.conv27_1_30_(relu25_)
        batchnorm27_1_30_ = self.batchnorm27_1_30_(conv27_1_30_)
        relu27_1_30_ = self.relu27_1_30_(batchnorm27_1_30_)
        conv28_1_30_padding = self.conv28_1_30_padding(relu27_1_30_)
        conv28_1_30_ = self.conv28_1_30_(conv28_1_30_padding)
        batchnorm28_1_30_ = self.batchnorm28_1_30_(conv28_1_30_)
        relu28_1_30_ = self.relu28_1_30_(batchnorm28_1_30_)
        conv29_1_30_ = self.conv29_1_30_(relu28_1_30_)
        batchnorm29_1_30_ = self.batchnorm29_1_30_(conv29_1_30_)
        conv27_1_31_ = self.conv27_1_31_(relu25_)
        batchnorm27_1_31_ = self.batchnorm27_1_31_(conv27_1_31_)
        relu27_1_31_ = self.relu27_1_31_(batchnorm27_1_31_)
        conv28_1_31_padding = self.conv28_1_31_padding(relu27_1_31_)
        conv28_1_31_ = self.conv28_1_31_(conv28_1_31_padding)
        batchnorm28_1_31_ = self.batchnorm28_1_31_(conv28_1_31_)
        relu28_1_31_ = self.relu28_1_31_(batchnorm28_1_31_)
        conv29_1_31_ = self.conv29_1_31_(relu28_1_31_)
        batchnorm29_1_31_ = self.batchnorm29_1_31_(conv29_1_31_)
        conv27_1_32_ = self.conv27_1_32_(relu25_)
        batchnorm27_1_32_ = self.batchnorm27_1_32_(conv27_1_32_)
        relu27_1_32_ = self.relu27_1_32_(batchnorm27_1_32_)
        conv28_1_32_padding = self.conv28_1_32_padding(relu27_1_32_)
        conv28_1_32_ = self.conv28_1_32_(conv28_1_32_padding)
        batchnorm28_1_32_ = self.batchnorm28_1_32_(conv28_1_32_)
        relu28_1_32_ = self.relu28_1_32_(batchnorm28_1_32_)
        conv29_1_32_ = self.conv29_1_32_(relu28_1_32_)
        batchnorm29_1_32_ = self.batchnorm29_1_32_(conv29_1_32_)
        add30_1_ = batchnorm29_1_1_ + batchnorm29_1_2_ + batchnorm29_1_3_ + batchnorm29_1_4_ + batchnorm29_1_5_ + batchnorm29_1_6_ + batchnorm29_1_7_ + batchnorm29_1_8_ + batchnorm29_1_9_ + batchnorm29_1_10_ + batchnorm29_1_11_ + batchnorm29_1_12_ + batchnorm29_1_13_ + batchnorm29_1_14_ + batchnorm29_1_15_ + batchnorm29_1_16_ + batchnorm29_1_17_ + batchnorm29_1_18_ + batchnorm29_1_19_ + batchnorm29_1_20_ + batchnorm29_1_21_ + batchnorm29_1_22_ + batchnorm29_1_23_ + batchnorm29_1_24_ + batchnorm29_1_25_ + batchnorm29_1_26_ + batchnorm29_1_27_ + batchnorm29_1_28_ + batchnorm29_1_29_ + batchnorm29_1_30_ + batchnorm29_1_31_ + batchnorm29_1_32_
        add31_ = add30_1_ + relu25_
        relu31_ = self.relu31_(add31_)
        conv33_1_1_ = self.conv33_1_1_(relu31_)
        batchnorm33_1_1_ = self.batchnorm33_1_1_(conv33_1_1_)
        relu33_1_1_ = self.relu33_1_1_(batchnorm33_1_1_)
        conv34_1_1_padding = self.conv34_1_1_padding(relu33_1_1_)
        conv34_1_1_ = self.conv34_1_1_(conv34_1_1_padding)
        batchnorm34_1_1_ = self.batchnorm34_1_1_(conv34_1_1_)
        relu34_1_1_ = self.relu34_1_1_(batchnorm34_1_1_)
        conv35_1_1_ = self.conv35_1_1_(relu34_1_1_)
        batchnorm35_1_1_ = self.batchnorm35_1_1_(conv35_1_1_)
        conv33_1_2_ = self.conv33_1_2_(relu31_)
        batchnorm33_1_2_ = self.batchnorm33_1_2_(conv33_1_2_)
        relu33_1_2_ = self.relu33_1_2_(batchnorm33_1_2_)
        conv34_1_2_padding = self.conv34_1_2_padding(relu33_1_2_)
        conv34_1_2_ = self.conv34_1_2_(conv34_1_2_padding)
        batchnorm34_1_2_ = self.batchnorm34_1_2_(conv34_1_2_)
        relu34_1_2_ = self.relu34_1_2_(batchnorm34_1_2_)
        conv35_1_2_ = self.conv35_1_2_(relu34_1_2_)
        batchnorm35_1_2_ = self.batchnorm35_1_2_(conv35_1_2_)
        conv33_1_3_ = self.conv33_1_3_(relu31_)
        batchnorm33_1_3_ = self.batchnorm33_1_3_(conv33_1_3_)
        relu33_1_3_ = self.relu33_1_3_(batchnorm33_1_3_)
        conv34_1_3_padding = self.conv34_1_3_padding(relu33_1_3_)
        conv34_1_3_ = self.conv34_1_3_(conv34_1_3_padding)
        batchnorm34_1_3_ = self.batchnorm34_1_3_(conv34_1_3_)
        relu34_1_3_ = self.relu34_1_3_(batchnorm34_1_3_)
        conv35_1_3_ = self.conv35_1_3_(relu34_1_3_)
        batchnorm35_1_3_ = self.batchnorm35_1_3_(conv35_1_3_)
        conv33_1_4_ = self.conv33_1_4_(relu31_)
        batchnorm33_1_4_ = self.batchnorm33_1_4_(conv33_1_4_)
        relu33_1_4_ = self.relu33_1_4_(batchnorm33_1_4_)
        conv34_1_4_padding = self.conv34_1_4_padding(relu33_1_4_)
        conv34_1_4_ = self.conv34_1_4_(conv34_1_4_padding)
        batchnorm34_1_4_ = self.batchnorm34_1_4_(conv34_1_4_)
        relu34_1_4_ = self.relu34_1_4_(batchnorm34_1_4_)
        conv35_1_4_ = self.conv35_1_4_(relu34_1_4_)
        batchnorm35_1_4_ = self.batchnorm35_1_4_(conv35_1_4_)
        conv33_1_5_ = self.conv33_1_5_(relu31_)
        batchnorm33_1_5_ = self.batchnorm33_1_5_(conv33_1_5_)
        relu33_1_5_ = self.relu33_1_5_(batchnorm33_1_5_)
        conv34_1_5_padding = self.conv34_1_5_padding(relu33_1_5_)
        conv34_1_5_ = self.conv34_1_5_(conv34_1_5_padding)
        batchnorm34_1_5_ = self.batchnorm34_1_5_(conv34_1_5_)
        relu34_1_5_ = self.relu34_1_5_(batchnorm34_1_5_)
        conv35_1_5_ = self.conv35_1_5_(relu34_1_5_)
        batchnorm35_1_5_ = self.batchnorm35_1_5_(conv35_1_5_)
        conv33_1_6_ = self.conv33_1_6_(relu31_)
        batchnorm33_1_6_ = self.batchnorm33_1_6_(conv33_1_6_)
        relu33_1_6_ = self.relu33_1_6_(batchnorm33_1_6_)
        conv34_1_6_padding = self.conv34_1_6_padding(relu33_1_6_)
        conv34_1_6_ = self.conv34_1_6_(conv34_1_6_padding)
        batchnorm34_1_6_ = self.batchnorm34_1_6_(conv34_1_6_)
        relu34_1_6_ = self.relu34_1_6_(batchnorm34_1_6_)
        conv35_1_6_ = self.conv35_1_6_(relu34_1_6_)
        batchnorm35_1_6_ = self.batchnorm35_1_6_(conv35_1_6_)
        conv33_1_7_ = self.conv33_1_7_(relu31_)
        batchnorm33_1_7_ = self.batchnorm33_1_7_(conv33_1_7_)
        relu33_1_7_ = self.relu33_1_7_(batchnorm33_1_7_)
        conv34_1_7_padding = self.conv34_1_7_padding(relu33_1_7_)
        conv34_1_7_ = self.conv34_1_7_(conv34_1_7_padding)
        batchnorm34_1_7_ = self.batchnorm34_1_7_(conv34_1_7_)
        relu34_1_7_ = self.relu34_1_7_(batchnorm34_1_7_)
        conv35_1_7_ = self.conv35_1_7_(relu34_1_7_)
        batchnorm35_1_7_ = self.batchnorm35_1_7_(conv35_1_7_)
        conv33_1_8_ = self.conv33_1_8_(relu31_)
        batchnorm33_1_8_ = self.batchnorm33_1_8_(conv33_1_8_)
        relu33_1_8_ = self.relu33_1_8_(batchnorm33_1_8_)
        conv34_1_8_padding = self.conv34_1_8_padding(relu33_1_8_)
        conv34_1_8_ = self.conv34_1_8_(conv34_1_8_padding)
        batchnorm34_1_8_ = self.batchnorm34_1_8_(conv34_1_8_)
        relu34_1_8_ = self.relu34_1_8_(batchnorm34_1_8_)
        conv35_1_8_ = self.conv35_1_8_(relu34_1_8_)
        batchnorm35_1_8_ = self.batchnorm35_1_8_(conv35_1_8_)
        conv33_1_9_ = self.conv33_1_9_(relu31_)
        batchnorm33_1_9_ = self.batchnorm33_1_9_(conv33_1_9_)
        relu33_1_9_ = self.relu33_1_9_(batchnorm33_1_9_)
        conv34_1_9_padding = self.conv34_1_9_padding(relu33_1_9_)
        conv34_1_9_ = self.conv34_1_9_(conv34_1_9_padding)
        batchnorm34_1_9_ = self.batchnorm34_1_9_(conv34_1_9_)
        relu34_1_9_ = self.relu34_1_9_(batchnorm34_1_9_)
        conv35_1_9_ = self.conv35_1_9_(relu34_1_9_)
        batchnorm35_1_9_ = self.batchnorm35_1_9_(conv35_1_9_)
        conv33_1_10_ = self.conv33_1_10_(relu31_)
        batchnorm33_1_10_ = self.batchnorm33_1_10_(conv33_1_10_)
        relu33_1_10_ = self.relu33_1_10_(batchnorm33_1_10_)
        conv34_1_10_padding = self.conv34_1_10_padding(relu33_1_10_)
        conv34_1_10_ = self.conv34_1_10_(conv34_1_10_padding)
        batchnorm34_1_10_ = self.batchnorm34_1_10_(conv34_1_10_)
        relu34_1_10_ = self.relu34_1_10_(batchnorm34_1_10_)
        conv35_1_10_ = self.conv35_1_10_(relu34_1_10_)
        batchnorm35_1_10_ = self.batchnorm35_1_10_(conv35_1_10_)
        conv33_1_11_ = self.conv33_1_11_(relu31_)
        batchnorm33_1_11_ = self.batchnorm33_1_11_(conv33_1_11_)
        relu33_1_11_ = self.relu33_1_11_(batchnorm33_1_11_)
        conv34_1_11_padding = self.conv34_1_11_padding(relu33_1_11_)
        conv34_1_11_ = self.conv34_1_11_(conv34_1_11_padding)
        batchnorm34_1_11_ = self.batchnorm34_1_11_(conv34_1_11_)
        relu34_1_11_ = self.relu34_1_11_(batchnorm34_1_11_)
        conv35_1_11_ = self.conv35_1_11_(relu34_1_11_)
        batchnorm35_1_11_ = self.batchnorm35_1_11_(conv35_1_11_)
        conv33_1_12_ = self.conv33_1_12_(relu31_)
        batchnorm33_1_12_ = self.batchnorm33_1_12_(conv33_1_12_)
        relu33_1_12_ = self.relu33_1_12_(batchnorm33_1_12_)
        conv34_1_12_padding = self.conv34_1_12_padding(relu33_1_12_)
        conv34_1_12_ = self.conv34_1_12_(conv34_1_12_padding)
        batchnorm34_1_12_ = self.batchnorm34_1_12_(conv34_1_12_)
        relu34_1_12_ = self.relu34_1_12_(batchnorm34_1_12_)
        conv35_1_12_ = self.conv35_1_12_(relu34_1_12_)
        batchnorm35_1_12_ = self.batchnorm35_1_12_(conv35_1_12_)
        conv33_1_13_ = self.conv33_1_13_(relu31_)
        batchnorm33_1_13_ = self.batchnorm33_1_13_(conv33_1_13_)
        relu33_1_13_ = self.relu33_1_13_(batchnorm33_1_13_)
        conv34_1_13_padding = self.conv34_1_13_padding(relu33_1_13_)
        conv34_1_13_ = self.conv34_1_13_(conv34_1_13_padding)
        batchnorm34_1_13_ = self.batchnorm34_1_13_(conv34_1_13_)
        relu34_1_13_ = self.relu34_1_13_(batchnorm34_1_13_)
        conv35_1_13_ = self.conv35_1_13_(relu34_1_13_)
        batchnorm35_1_13_ = self.batchnorm35_1_13_(conv35_1_13_)
        conv33_1_14_ = self.conv33_1_14_(relu31_)
        batchnorm33_1_14_ = self.batchnorm33_1_14_(conv33_1_14_)
        relu33_1_14_ = self.relu33_1_14_(batchnorm33_1_14_)
        conv34_1_14_padding = self.conv34_1_14_padding(relu33_1_14_)
        conv34_1_14_ = self.conv34_1_14_(conv34_1_14_padding)
        batchnorm34_1_14_ = self.batchnorm34_1_14_(conv34_1_14_)
        relu34_1_14_ = self.relu34_1_14_(batchnorm34_1_14_)
        conv35_1_14_ = self.conv35_1_14_(relu34_1_14_)
        batchnorm35_1_14_ = self.batchnorm35_1_14_(conv35_1_14_)
        conv33_1_15_ = self.conv33_1_15_(relu31_)
        batchnorm33_1_15_ = self.batchnorm33_1_15_(conv33_1_15_)
        relu33_1_15_ = self.relu33_1_15_(batchnorm33_1_15_)
        conv34_1_15_padding = self.conv34_1_15_padding(relu33_1_15_)
        conv34_1_15_ = self.conv34_1_15_(conv34_1_15_padding)
        batchnorm34_1_15_ = self.batchnorm34_1_15_(conv34_1_15_)
        relu34_1_15_ = self.relu34_1_15_(batchnorm34_1_15_)
        conv35_1_15_ = self.conv35_1_15_(relu34_1_15_)
        batchnorm35_1_15_ = self.batchnorm35_1_15_(conv35_1_15_)
        conv33_1_16_ = self.conv33_1_16_(relu31_)
        batchnorm33_1_16_ = self.batchnorm33_1_16_(conv33_1_16_)
        relu33_1_16_ = self.relu33_1_16_(batchnorm33_1_16_)
        conv34_1_16_padding = self.conv34_1_16_padding(relu33_1_16_)
        conv34_1_16_ = self.conv34_1_16_(conv34_1_16_padding)
        batchnorm34_1_16_ = self.batchnorm34_1_16_(conv34_1_16_)
        relu34_1_16_ = self.relu34_1_16_(batchnorm34_1_16_)
        conv35_1_16_ = self.conv35_1_16_(relu34_1_16_)
        batchnorm35_1_16_ = self.batchnorm35_1_16_(conv35_1_16_)
        conv33_1_17_ = self.conv33_1_17_(relu31_)
        batchnorm33_1_17_ = self.batchnorm33_1_17_(conv33_1_17_)
        relu33_1_17_ = self.relu33_1_17_(batchnorm33_1_17_)
        conv34_1_17_padding = self.conv34_1_17_padding(relu33_1_17_)
        conv34_1_17_ = self.conv34_1_17_(conv34_1_17_padding)
        batchnorm34_1_17_ = self.batchnorm34_1_17_(conv34_1_17_)
        relu34_1_17_ = self.relu34_1_17_(batchnorm34_1_17_)
        conv35_1_17_ = self.conv35_1_17_(relu34_1_17_)
        batchnorm35_1_17_ = self.batchnorm35_1_17_(conv35_1_17_)
        conv33_1_18_ = self.conv33_1_18_(relu31_)
        batchnorm33_1_18_ = self.batchnorm33_1_18_(conv33_1_18_)
        relu33_1_18_ = self.relu33_1_18_(batchnorm33_1_18_)
        conv34_1_18_padding = self.conv34_1_18_padding(relu33_1_18_)
        conv34_1_18_ = self.conv34_1_18_(conv34_1_18_padding)
        batchnorm34_1_18_ = self.batchnorm34_1_18_(conv34_1_18_)
        relu34_1_18_ = self.relu34_1_18_(batchnorm34_1_18_)
        conv35_1_18_ = self.conv35_1_18_(relu34_1_18_)
        batchnorm35_1_18_ = self.batchnorm35_1_18_(conv35_1_18_)
        conv33_1_19_ = self.conv33_1_19_(relu31_)
        batchnorm33_1_19_ = self.batchnorm33_1_19_(conv33_1_19_)
        relu33_1_19_ = self.relu33_1_19_(batchnorm33_1_19_)
        conv34_1_19_padding = self.conv34_1_19_padding(relu33_1_19_)
        conv34_1_19_ = self.conv34_1_19_(conv34_1_19_padding)
        batchnorm34_1_19_ = self.batchnorm34_1_19_(conv34_1_19_)
        relu34_1_19_ = self.relu34_1_19_(batchnorm34_1_19_)
        conv35_1_19_ = self.conv35_1_19_(relu34_1_19_)
        batchnorm35_1_19_ = self.batchnorm35_1_19_(conv35_1_19_)
        conv33_1_20_ = self.conv33_1_20_(relu31_)
        batchnorm33_1_20_ = self.batchnorm33_1_20_(conv33_1_20_)
        relu33_1_20_ = self.relu33_1_20_(batchnorm33_1_20_)
        conv34_1_20_padding = self.conv34_1_20_padding(relu33_1_20_)
        conv34_1_20_ = self.conv34_1_20_(conv34_1_20_padding)
        batchnorm34_1_20_ = self.batchnorm34_1_20_(conv34_1_20_)
        relu34_1_20_ = self.relu34_1_20_(batchnorm34_1_20_)
        conv35_1_20_ = self.conv35_1_20_(relu34_1_20_)
        batchnorm35_1_20_ = self.batchnorm35_1_20_(conv35_1_20_)
        conv33_1_21_ = self.conv33_1_21_(relu31_)
        batchnorm33_1_21_ = self.batchnorm33_1_21_(conv33_1_21_)
        relu33_1_21_ = self.relu33_1_21_(batchnorm33_1_21_)
        conv34_1_21_padding = self.conv34_1_21_padding(relu33_1_21_)
        conv34_1_21_ = self.conv34_1_21_(conv34_1_21_padding)
        batchnorm34_1_21_ = self.batchnorm34_1_21_(conv34_1_21_)
        relu34_1_21_ = self.relu34_1_21_(batchnorm34_1_21_)
        conv35_1_21_ = self.conv35_1_21_(relu34_1_21_)
        batchnorm35_1_21_ = self.batchnorm35_1_21_(conv35_1_21_)
        conv33_1_22_ = self.conv33_1_22_(relu31_)
        batchnorm33_1_22_ = self.batchnorm33_1_22_(conv33_1_22_)
        relu33_1_22_ = self.relu33_1_22_(batchnorm33_1_22_)
        conv34_1_22_padding = self.conv34_1_22_padding(relu33_1_22_)
        conv34_1_22_ = self.conv34_1_22_(conv34_1_22_padding)
        batchnorm34_1_22_ = self.batchnorm34_1_22_(conv34_1_22_)
        relu34_1_22_ = self.relu34_1_22_(batchnorm34_1_22_)
        conv35_1_22_ = self.conv35_1_22_(relu34_1_22_)
        batchnorm35_1_22_ = self.batchnorm35_1_22_(conv35_1_22_)
        conv33_1_23_ = self.conv33_1_23_(relu31_)
        batchnorm33_1_23_ = self.batchnorm33_1_23_(conv33_1_23_)
        relu33_1_23_ = self.relu33_1_23_(batchnorm33_1_23_)
        conv34_1_23_padding = self.conv34_1_23_padding(relu33_1_23_)
        conv34_1_23_ = self.conv34_1_23_(conv34_1_23_padding)
        batchnorm34_1_23_ = self.batchnorm34_1_23_(conv34_1_23_)
        relu34_1_23_ = self.relu34_1_23_(batchnorm34_1_23_)
        conv35_1_23_ = self.conv35_1_23_(relu34_1_23_)
        batchnorm35_1_23_ = self.batchnorm35_1_23_(conv35_1_23_)
        conv33_1_24_ = self.conv33_1_24_(relu31_)
        batchnorm33_1_24_ = self.batchnorm33_1_24_(conv33_1_24_)
        relu33_1_24_ = self.relu33_1_24_(batchnorm33_1_24_)
        conv34_1_24_padding = self.conv34_1_24_padding(relu33_1_24_)
        conv34_1_24_ = self.conv34_1_24_(conv34_1_24_padding)
        batchnorm34_1_24_ = self.batchnorm34_1_24_(conv34_1_24_)
        relu34_1_24_ = self.relu34_1_24_(batchnorm34_1_24_)
        conv35_1_24_ = self.conv35_1_24_(relu34_1_24_)
        batchnorm35_1_24_ = self.batchnorm35_1_24_(conv35_1_24_)
        conv33_1_25_ = self.conv33_1_25_(relu31_)
        batchnorm33_1_25_ = self.batchnorm33_1_25_(conv33_1_25_)
        relu33_1_25_ = self.relu33_1_25_(batchnorm33_1_25_)
        conv34_1_25_padding = self.conv34_1_25_padding(relu33_1_25_)
        conv34_1_25_ = self.conv34_1_25_(conv34_1_25_padding)
        batchnorm34_1_25_ = self.batchnorm34_1_25_(conv34_1_25_)
        relu34_1_25_ = self.relu34_1_25_(batchnorm34_1_25_)
        conv35_1_25_ = self.conv35_1_25_(relu34_1_25_)
        batchnorm35_1_25_ = self.batchnorm35_1_25_(conv35_1_25_)
        conv33_1_26_ = self.conv33_1_26_(relu31_)
        batchnorm33_1_26_ = self.batchnorm33_1_26_(conv33_1_26_)
        relu33_1_26_ = self.relu33_1_26_(batchnorm33_1_26_)
        conv34_1_26_padding = self.conv34_1_26_padding(relu33_1_26_)
        conv34_1_26_ = self.conv34_1_26_(conv34_1_26_padding)
        batchnorm34_1_26_ = self.batchnorm34_1_26_(conv34_1_26_)
        relu34_1_26_ = self.relu34_1_26_(batchnorm34_1_26_)
        conv35_1_26_ = self.conv35_1_26_(relu34_1_26_)
        batchnorm35_1_26_ = self.batchnorm35_1_26_(conv35_1_26_)
        conv33_1_27_ = self.conv33_1_27_(relu31_)
        batchnorm33_1_27_ = self.batchnorm33_1_27_(conv33_1_27_)
        relu33_1_27_ = self.relu33_1_27_(batchnorm33_1_27_)
        conv34_1_27_padding = self.conv34_1_27_padding(relu33_1_27_)
        conv34_1_27_ = self.conv34_1_27_(conv34_1_27_padding)
        batchnorm34_1_27_ = self.batchnorm34_1_27_(conv34_1_27_)
        relu34_1_27_ = self.relu34_1_27_(batchnorm34_1_27_)
        conv35_1_27_ = self.conv35_1_27_(relu34_1_27_)
        batchnorm35_1_27_ = self.batchnorm35_1_27_(conv35_1_27_)
        conv33_1_28_ = self.conv33_1_28_(relu31_)
        batchnorm33_1_28_ = self.batchnorm33_1_28_(conv33_1_28_)
        relu33_1_28_ = self.relu33_1_28_(batchnorm33_1_28_)
        conv34_1_28_padding = self.conv34_1_28_padding(relu33_1_28_)
        conv34_1_28_ = self.conv34_1_28_(conv34_1_28_padding)
        batchnorm34_1_28_ = self.batchnorm34_1_28_(conv34_1_28_)
        relu34_1_28_ = self.relu34_1_28_(batchnorm34_1_28_)
        conv35_1_28_ = self.conv35_1_28_(relu34_1_28_)
        batchnorm35_1_28_ = self.batchnorm35_1_28_(conv35_1_28_)
        conv33_1_29_ = self.conv33_1_29_(relu31_)
        batchnorm33_1_29_ = self.batchnorm33_1_29_(conv33_1_29_)
        relu33_1_29_ = self.relu33_1_29_(batchnorm33_1_29_)
        conv34_1_29_padding = self.conv34_1_29_padding(relu33_1_29_)
        conv34_1_29_ = self.conv34_1_29_(conv34_1_29_padding)
        batchnorm34_1_29_ = self.batchnorm34_1_29_(conv34_1_29_)
        relu34_1_29_ = self.relu34_1_29_(batchnorm34_1_29_)
        conv35_1_29_ = self.conv35_1_29_(relu34_1_29_)
        batchnorm35_1_29_ = self.batchnorm35_1_29_(conv35_1_29_)
        conv33_1_30_ = self.conv33_1_30_(relu31_)
        batchnorm33_1_30_ = self.batchnorm33_1_30_(conv33_1_30_)
        relu33_1_30_ = self.relu33_1_30_(batchnorm33_1_30_)
        conv34_1_30_padding = self.conv34_1_30_padding(relu33_1_30_)
        conv34_1_30_ = self.conv34_1_30_(conv34_1_30_padding)
        batchnorm34_1_30_ = self.batchnorm34_1_30_(conv34_1_30_)
        relu34_1_30_ = self.relu34_1_30_(batchnorm34_1_30_)
        conv35_1_30_ = self.conv35_1_30_(relu34_1_30_)
        batchnorm35_1_30_ = self.batchnorm35_1_30_(conv35_1_30_)
        conv33_1_31_ = self.conv33_1_31_(relu31_)
        batchnorm33_1_31_ = self.batchnorm33_1_31_(conv33_1_31_)
        relu33_1_31_ = self.relu33_1_31_(batchnorm33_1_31_)
        conv34_1_31_padding = self.conv34_1_31_padding(relu33_1_31_)
        conv34_1_31_ = self.conv34_1_31_(conv34_1_31_padding)
        batchnorm34_1_31_ = self.batchnorm34_1_31_(conv34_1_31_)
        relu34_1_31_ = self.relu34_1_31_(batchnorm34_1_31_)
        conv35_1_31_ = self.conv35_1_31_(relu34_1_31_)
        batchnorm35_1_31_ = self.batchnorm35_1_31_(conv35_1_31_)
        conv33_1_32_ = self.conv33_1_32_(relu31_)
        batchnorm33_1_32_ = self.batchnorm33_1_32_(conv33_1_32_)
        relu33_1_32_ = self.relu33_1_32_(batchnorm33_1_32_)
        conv34_1_32_padding = self.conv34_1_32_padding(relu33_1_32_)
        conv34_1_32_ = self.conv34_1_32_(conv34_1_32_padding)
        batchnorm34_1_32_ = self.batchnorm34_1_32_(conv34_1_32_)
        relu34_1_32_ = self.relu34_1_32_(batchnorm34_1_32_)
        conv35_1_32_ = self.conv35_1_32_(relu34_1_32_)
        batchnorm35_1_32_ = self.batchnorm35_1_32_(conv35_1_32_)
        add36_1_ = batchnorm35_1_1_ + batchnorm35_1_2_ + batchnorm35_1_3_ + batchnorm35_1_4_ + batchnorm35_1_5_ + batchnorm35_1_6_ + batchnorm35_1_7_ + batchnorm35_1_8_ + batchnorm35_1_9_ + batchnorm35_1_10_ + batchnorm35_1_11_ + batchnorm35_1_12_ + batchnorm35_1_13_ + batchnorm35_1_14_ + batchnorm35_1_15_ + batchnorm35_1_16_ + batchnorm35_1_17_ + batchnorm35_1_18_ + batchnorm35_1_19_ + batchnorm35_1_20_ + batchnorm35_1_21_ + batchnorm35_1_22_ + batchnorm35_1_23_ + batchnorm35_1_24_ + batchnorm35_1_25_ + batchnorm35_1_26_ + batchnorm35_1_27_ + batchnorm35_1_28_ + batchnorm35_1_29_ + batchnorm35_1_30_ + batchnorm35_1_31_ + batchnorm35_1_32_
        add37_ = add36_1_ + relu31_
        relu37_ = self.relu37_(add37_)
        conv39_1_1_ = self.conv39_1_1_(relu37_)
        batchnorm39_1_1_ = self.batchnorm39_1_1_(conv39_1_1_)
        relu39_1_1_ = self.relu39_1_1_(batchnorm39_1_1_)
        conv40_1_1_padding = self.conv40_1_1_padding(relu39_1_1_)
        conv40_1_1_ = self.conv40_1_1_(conv40_1_1_padding)
        batchnorm40_1_1_ = self.batchnorm40_1_1_(conv40_1_1_)
        relu40_1_1_ = self.relu40_1_1_(batchnorm40_1_1_)
        conv41_1_1_ = self.conv41_1_1_(relu40_1_1_)
        batchnorm41_1_1_ = self.batchnorm41_1_1_(conv41_1_1_)
        conv39_1_2_ = self.conv39_1_2_(relu37_)
        batchnorm39_1_2_ = self.batchnorm39_1_2_(conv39_1_2_)
        relu39_1_2_ = self.relu39_1_2_(batchnorm39_1_2_)
        conv40_1_2_padding = self.conv40_1_2_padding(relu39_1_2_)
        conv40_1_2_ = self.conv40_1_2_(conv40_1_2_padding)
        batchnorm40_1_2_ = self.batchnorm40_1_2_(conv40_1_2_)
        relu40_1_2_ = self.relu40_1_2_(batchnorm40_1_2_)
        conv41_1_2_ = self.conv41_1_2_(relu40_1_2_)
        batchnorm41_1_2_ = self.batchnorm41_1_2_(conv41_1_2_)
        conv39_1_3_ = self.conv39_1_3_(relu37_)
        batchnorm39_1_3_ = self.batchnorm39_1_3_(conv39_1_3_)
        relu39_1_3_ = self.relu39_1_3_(batchnorm39_1_3_)
        conv40_1_3_padding = self.conv40_1_3_padding(relu39_1_3_)
        conv40_1_3_ = self.conv40_1_3_(conv40_1_3_padding)
        batchnorm40_1_3_ = self.batchnorm40_1_3_(conv40_1_3_)
        relu40_1_3_ = self.relu40_1_3_(batchnorm40_1_3_)
        conv41_1_3_ = self.conv41_1_3_(relu40_1_3_)
        batchnorm41_1_3_ = self.batchnorm41_1_3_(conv41_1_3_)
        conv39_1_4_ = self.conv39_1_4_(relu37_)
        batchnorm39_1_4_ = self.batchnorm39_1_4_(conv39_1_4_)
        relu39_1_4_ = self.relu39_1_4_(batchnorm39_1_4_)
        conv40_1_4_padding = self.conv40_1_4_padding(relu39_1_4_)
        conv40_1_4_ = self.conv40_1_4_(conv40_1_4_padding)
        batchnorm40_1_4_ = self.batchnorm40_1_4_(conv40_1_4_)
        relu40_1_4_ = self.relu40_1_4_(batchnorm40_1_4_)
        conv41_1_4_ = self.conv41_1_4_(relu40_1_4_)
        batchnorm41_1_4_ = self.batchnorm41_1_4_(conv41_1_4_)
        conv39_1_5_ = self.conv39_1_5_(relu37_)
        batchnorm39_1_5_ = self.batchnorm39_1_5_(conv39_1_5_)
        relu39_1_5_ = self.relu39_1_5_(batchnorm39_1_5_)
        conv40_1_5_padding = self.conv40_1_5_padding(relu39_1_5_)
        conv40_1_5_ = self.conv40_1_5_(conv40_1_5_padding)
        batchnorm40_1_5_ = self.batchnorm40_1_5_(conv40_1_5_)
        relu40_1_5_ = self.relu40_1_5_(batchnorm40_1_5_)
        conv41_1_5_ = self.conv41_1_5_(relu40_1_5_)
        batchnorm41_1_5_ = self.batchnorm41_1_5_(conv41_1_5_)
        conv39_1_6_ = self.conv39_1_6_(relu37_)
        batchnorm39_1_6_ = self.batchnorm39_1_6_(conv39_1_6_)
        relu39_1_6_ = self.relu39_1_6_(batchnorm39_1_6_)
        conv40_1_6_padding = self.conv40_1_6_padding(relu39_1_6_)
        conv40_1_6_ = self.conv40_1_6_(conv40_1_6_padding)
        batchnorm40_1_6_ = self.batchnorm40_1_6_(conv40_1_6_)
        relu40_1_6_ = self.relu40_1_6_(batchnorm40_1_6_)
        conv41_1_6_ = self.conv41_1_6_(relu40_1_6_)
        batchnorm41_1_6_ = self.batchnorm41_1_6_(conv41_1_6_)
        conv39_1_7_ = self.conv39_1_7_(relu37_)
        batchnorm39_1_7_ = self.batchnorm39_1_7_(conv39_1_7_)
        relu39_1_7_ = self.relu39_1_7_(batchnorm39_1_7_)
        conv40_1_7_padding = self.conv40_1_7_padding(relu39_1_7_)
        conv40_1_7_ = self.conv40_1_7_(conv40_1_7_padding)
        batchnorm40_1_7_ = self.batchnorm40_1_7_(conv40_1_7_)
        relu40_1_7_ = self.relu40_1_7_(batchnorm40_1_7_)
        conv41_1_7_ = self.conv41_1_7_(relu40_1_7_)
        batchnorm41_1_7_ = self.batchnorm41_1_7_(conv41_1_7_)
        conv39_1_8_ = self.conv39_1_8_(relu37_)
        batchnorm39_1_8_ = self.batchnorm39_1_8_(conv39_1_8_)
        relu39_1_8_ = self.relu39_1_8_(batchnorm39_1_8_)
        conv40_1_8_padding = self.conv40_1_8_padding(relu39_1_8_)
        conv40_1_8_ = self.conv40_1_8_(conv40_1_8_padding)
        batchnorm40_1_8_ = self.batchnorm40_1_8_(conv40_1_8_)
        relu40_1_8_ = self.relu40_1_8_(batchnorm40_1_8_)
        conv41_1_8_ = self.conv41_1_8_(relu40_1_8_)
        batchnorm41_1_8_ = self.batchnorm41_1_8_(conv41_1_8_)
        conv39_1_9_ = self.conv39_1_9_(relu37_)
        batchnorm39_1_9_ = self.batchnorm39_1_9_(conv39_1_9_)
        relu39_1_9_ = self.relu39_1_9_(batchnorm39_1_9_)
        conv40_1_9_padding = self.conv40_1_9_padding(relu39_1_9_)
        conv40_1_9_ = self.conv40_1_9_(conv40_1_9_padding)
        batchnorm40_1_9_ = self.batchnorm40_1_9_(conv40_1_9_)
        relu40_1_9_ = self.relu40_1_9_(batchnorm40_1_9_)
        conv41_1_9_ = self.conv41_1_9_(relu40_1_9_)
        batchnorm41_1_9_ = self.batchnorm41_1_9_(conv41_1_9_)
        conv39_1_10_ = self.conv39_1_10_(relu37_)
        batchnorm39_1_10_ = self.batchnorm39_1_10_(conv39_1_10_)
        relu39_1_10_ = self.relu39_1_10_(batchnorm39_1_10_)
        conv40_1_10_padding = self.conv40_1_10_padding(relu39_1_10_)
        conv40_1_10_ = self.conv40_1_10_(conv40_1_10_padding)
        batchnorm40_1_10_ = self.batchnorm40_1_10_(conv40_1_10_)
        relu40_1_10_ = self.relu40_1_10_(batchnorm40_1_10_)
        conv41_1_10_ = self.conv41_1_10_(relu40_1_10_)
        batchnorm41_1_10_ = self.batchnorm41_1_10_(conv41_1_10_)
        conv39_1_11_ = self.conv39_1_11_(relu37_)
        batchnorm39_1_11_ = self.batchnorm39_1_11_(conv39_1_11_)
        relu39_1_11_ = self.relu39_1_11_(batchnorm39_1_11_)
        conv40_1_11_padding = self.conv40_1_11_padding(relu39_1_11_)
        conv40_1_11_ = self.conv40_1_11_(conv40_1_11_padding)
        batchnorm40_1_11_ = self.batchnorm40_1_11_(conv40_1_11_)
        relu40_1_11_ = self.relu40_1_11_(batchnorm40_1_11_)
        conv41_1_11_ = self.conv41_1_11_(relu40_1_11_)
        batchnorm41_1_11_ = self.batchnorm41_1_11_(conv41_1_11_)
        conv39_1_12_ = self.conv39_1_12_(relu37_)
        batchnorm39_1_12_ = self.batchnorm39_1_12_(conv39_1_12_)
        relu39_1_12_ = self.relu39_1_12_(batchnorm39_1_12_)
        conv40_1_12_padding = self.conv40_1_12_padding(relu39_1_12_)
        conv40_1_12_ = self.conv40_1_12_(conv40_1_12_padding)
        batchnorm40_1_12_ = self.batchnorm40_1_12_(conv40_1_12_)
        relu40_1_12_ = self.relu40_1_12_(batchnorm40_1_12_)
        conv41_1_12_ = self.conv41_1_12_(relu40_1_12_)
        batchnorm41_1_12_ = self.batchnorm41_1_12_(conv41_1_12_)
        conv39_1_13_ = self.conv39_1_13_(relu37_)
        batchnorm39_1_13_ = self.batchnorm39_1_13_(conv39_1_13_)
        relu39_1_13_ = self.relu39_1_13_(batchnorm39_1_13_)
        conv40_1_13_padding = self.conv40_1_13_padding(relu39_1_13_)
        conv40_1_13_ = self.conv40_1_13_(conv40_1_13_padding)
        batchnorm40_1_13_ = self.batchnorm40_1_13_(conv40_1_13_)
        relu40_1_13_ = self.relu40_1_13_(batchnorm40_1_13_)
        conv41_1_13_ = self.conv41_1_13_(relu40_1_13_)
        batchnorm41_1_13_ = self.batchnorm41_1_13_(conv41_1_13_)
        conv39_1_14_ = self.conv39_1_14_(relu37_)
        batchnorm39_1_14_ = self.batchnorm39_1_14_(conv39_1_14_)
        relu39_1_14_ = self.relu39_1_14_(batchnorm39_1_14_)
        conv40_1_14_padding = self.conv40_1_14_padding(relu39_1_14_)
        conv40_1_14_ = self.conv40_1_14_(conv40_1_14_padding)
        batchnorm40_1_14_ = self.batchnorm40_1_14_(conv40_1_14_)
        relu40_1_14_ = self.relu40_1_14_(batchnorm40_1_14_)
        conv41_1_14_ = self.conv41_1_14_(relu40_1_14_)
        batchnorm41_1_14_ = self.batchnorm41_1_14_(conv41_1_14_)
        conv39_1_15_ = self.conv39_1_15_(relu37_)
        batchnorm39_1_15_ = self.batchnorm39_1_15_(conv39_1_15_)
        relu39_1_15_ = self.relu39_1_15_(batchnorm39_1_15_)
        conv40_1_15_padding = self.conv40_1_15_padding(relu39_1_15_)
        conv40_1_15_ = self.conv40_1_15_(conv40_1_15_padding)
        batchnorm40_1_15_ = self.batchnorm40_1_15_(conv40_1_15_)
        relu40_1_15_ = self.relu40_1_15_(batchnorm40_1_15_)
        conv41_1_15_ = self.conv41_1_15_(relu40_1_15_)
        batchnorm41_1_15_ = self.batchnorm41_1_15_(conv41_1_15_)
        conv39_1_16_ = self.conv39_1_16_(relu37_)
        batchnorm39_1_16_ = self.batchnorm39_1_16_(conv39_1_16_)
        relu39_1_16_ = self.relu39_1_16_(batchnorm39_1_16_)
        conv40_1_16_padding = self.conv40_1_16_padding(relu39_1_16_)
        conv40_1_16_ = self.conv40_1_16_(conv40_1_16_padding)
        batchnorm40_1_16_ = self.batchnorm40_1_16_(conv40_1_16_)
        relu40_1_16_ = self.relu40_1_16_(batchnorm40_1_16_)
        conv41_1_16_ = self.conv41_1_16_(relu40_1_16_)
        batchnorm41_1_16_ = self.batchnorm41_1_16_(conv41_1_16_)
        conv39_1_17_ = self.conv39_1_17_(relu37_)
        batchnorm39_1_17_ = self.batchnorm39_1_17_(conv39_1_17_)
        relu39_1_17_ = self.relu39_1_17_(batchnorm39_1_17_)
        conv40_1_17_padding = self.conv40_1_17_padding(relu39_1_17_)
        conv40_1_17_ = self.conv40_1_17_(conv40_1_17_padding)
        batchnorm40_1_17_ = self.batchnorm40_1_17_(conv40_1_17_)
        relu40_1_17_ = self.relu40_1_17_(batchnorm40_1_17_)
        conv41_1_17_ = self.conv41_1_17_(relu40_1_17_)
        batchnorm41_1_17_ = self.batchnorm41_1_17_(conv41_1_17_)
        conv39_1_18_ = self.conv39_1_18_(relu37_)
        batchnorm39_1_18_ = self.batchnorm39_1_18_(conv39_1_18_)
        relu39_1_18_ = self.relu39_1_18_(batchnorm39_1_18_)
        conv40_1_18_padding = self.conv40_1_18_padding(relu39_1_18_)
        conv40_1_18_ = self.conv40_1_18_(conv40_1_18_padding)
        batchnorm40_1_18_ = self.batchnorm40_1_18_(conv40_1_18_)
        relu40_1_18_ = self.relu40_1_18_(batchnorm40_1_18_)
        conv41_1_18_ = self.conv41_1_18_(relu40_1_18_)
        batchnorm41_1_18_ = self.batchnorm41_1_18_(conv41_1_18_)
        conv39_1_19_ = self.conv39_1_19_(relu37_)
        batchnorm39_1_19_ = self.batchnorm39_1_19_(conv39_1_19_)
        relu39_1_19_ = self.relu39_1_19_(batchnorm39_1_19_)
        conv40_1_19_padding = self.conv40_1_19_padding(relu39_1_19_)
        conv40_1_19_ = self.conv40_1_19_(conv40_1_19_padding)
        batchnorm40_1_19_ = self.batchnorm40_1_19_(conv40_1_19_)
        relu40_1_19_ = self.relu40_1_19_(batchnorm40_1_19_)
        conv41_1_19_ = self.conv41_1_19_(relu40_1_19_)
        batchnorm41_1_19_ = self.batchnorm41_1_19_(conv41_1_19_)
        conv39_1_20_ = self.conv39_1_20_(relu37_)
        batchnorm39_1_20_ = self.batchnorm39_1_20_(conv39_1_20_)
        relu39_1_20_ = self.relu39_1_20_(batchnorm39_1_20_)
        conv40_1_20_padding = self.conv40_1_20_padding(relu39_1_20_)
        conv40_1_20_ = self.conv40_1_20_(conv40_1_20_padding)
        batchnorm40_1_20_ = self.batchnorm40_1_20_(conv40_1_20_)
        relu40_1_20_ = self.relu40_1_20_(batchnorm40_1_20_)
        conv41_1_20_ = self.conv41_1_20_(relu40_1_20_)
        batchnorm41_1_20_ = self.batchnorm41_1_20_(conv41_1_20_)
        conv39_1_21_ = self.conv39_1_21_(relu37_)
        batchnorm39_1_21_ = self.batchnorm39_1_21_(conv39_1_21_)
        relu39_1_21_ = self.relu39_1_21_(batchnorm39_1_21_)
        conv40_1_21_padding = self.conv40_1_21_padding(relu39_1_21_)
        conv40_1_21_ = self.conv40_1_21_(conv40_1_21_padding)
        batchnorm40_1_21_ = self.batchnorm40_1_21_(conv40_1_21_)
        relu40_1_21_ = self.relu40_1_21_(batchnorm40_1_21_)
        conv41_1_21_ = self.conv41_1_21_(relu40_1_21_)
        batchnorm41_1_21_ = self.batchnorm41_1_21_(conv41_1_21_)
        conv39_1_22_ = self.conv39_1_22_(relu37_)
        batchnorm39_1_22_ = self.batchnorm39_1_22_(conv39_1_22_)
        relu39_1_22_ = self.relu39_1_22_(batchnorm39_1_22_)
        conv40_1_22_padding = self.conv40_1_22_padding(relu39_1_22_)
        conv40_1_22_ = self.conv40_1_22_(conv40_1_22_padding)
        batchnorm40_1_22_ = self.batchnorm40_1_22_(conv40_1_22_)
        relu40_1_22_ = self.relu40_1_22_(batchnorm40_1_22_)
        conv41_1_22_ = self.conv41_1_22_(relu40_1_22_)
        batchnorm41_1_22_ = self.batchnorm41_1_22_(conv41_1_22_)
        conv39_1_23_ = self.conv39_1_23_(relu37_)
        batchnorm39_1_23_ = self.batchnorm39_1_23_(conv39_1_23_)
        relu39_1_23_ = self.relu39_1_23_(batchnorm39_1_23_)
        conv40_1_23_padding = self.conv40_1_23_padding(relu39_1_23_)
        conv40_1_23_ = self.conv40_1_23_(conv40_1_23_padding)
        batchnorm40_1_23_ = self.batchnorm40_1_23_(conv40_1_23_)
        relu40_1_23_ = self.relu40_1_23_(batchnorm40_1_23_)
        conv41_1_23_ = self.conv41_1_23_(relu40_1_23_)
        batchnorm41_1_23_ = self.batchnorm41_1_23_(conv41_1_23_)
        conv39_1_24_ = self.conv39_1_24_(relu37_)
        batchnorm39_1_24_ = self.batchnorm39_1_24_(conv39_1_24_)
        relu39_1_24_ = self.relu39_1_24_(batchnorm39_1_24_)
        conv40_1_24_padding = self.conv40_1_24_padding(relu39_1_24_)
        conv40_1_24_ = self.conv40_1_24_(conv40_1_24_padding)
        batchnorm40_1_24_ = self.batchnorm40_1_24_(conv40_1_24_)
        relu40_1_24_ = self.relu40_1_24_(batchnorm40_1_24_)
        conv41_1_24_ = self.conv41_1_24_(relu40_1_24_)
        batchnorm41_1_24_ = self.batchnorm41_1_24_(conv41_1_24_)
        conv39_1_25_ = self.conv39_1_25_(relu37_)
        batchnorm39_1_25_ = self.batchnorm39_1_25_(conv39_1_25_)
        relu39_1_25_ = self.relu39_1_25_(batchnorm39_1_25_)
        conv40_1_25_padding = self.conv40_1_25_padding(relu39_1_25_)
        conv40_1_25_ = self.conv40_1_25_(conv40_1_25_padding)
        batchnorm40_1_25_ = self.batchnorm40_1_25_(conv40_1_25_)
        relu40_1_25_ = self.relu40_1_25_(batchnorm40_1_25_)
        conv41_1_25_ = self.conv41_1_25_(relu40_1_25_)
        batchnorm41_1_25_ = self.batchnorm41_1_25_(conv41_1_25_)
        conv39_1_26_ = self.conv39_1_26_(relu37_)
        batchnorm39_1_26_ = self.batchnorm39_1_26_(conv39_1_26_)
        relu39_1_26_ = self.relu39_1_26_(batchnorm39_1_26_)
        conv40_1_26_padding = self.conv40_1_26_padding(relu39_1_26_)
        conv40_1_26_ = self.conv40_1_26_(conv40_1_26_padding)
        batchnorm40_1_26_ = self.batchnorm40_1_26_(conv40_1_26_)
        relu40_1_26_ = self.relu40_1_26_(batchnorm40_1_26_)
        conv41_1_26_ = self.conv41_1_26_(relu40_1_26_)
        batchnorm41_1_26_ = self.batchnorm41_1_26_(conv41_1_26_)
        conv39_1_27_ = self.conv39_1_27_(relu37_)
        batchnorm39_1_27_ = self.batchnorm39_1_27_(conv39_1_27_)
        relu39_1_27_ = self.relu39_1_27_(batchnorm39_1_27_)
        conv40_1_27_padding = self.conv40_1_27_padding(relu39_1_27_)
        conv40_1_27_ = self.conv40_1_27_(conv40_1_27_padding)
        batchnorm40_1_27_ = self.batchnorm40_1_27_(conv40_1_27_)
        relu40_1_27_ = self.relu40_1_27_(batchnorm40_1_27_)
        conv41_1_27_ = self.conv41_1_27_(relu40_1_27_)
        batchnorm41_1_27_ = self.batchnorm41_1_27_(conv41_1_27_)
        conv39_1_28_ = self.conv39_1_28_(relu37_)
        batchnorm39_1_28_ = self.batchnorm39_1_28_(conv39_1_28_)
        relu39_1_28_ = self.relu39_1_28_(batchnorm39_1_28_)
        conv40_1_28_padding = self.conv40_1_28_padding(relu39_1_28_)
        conv40_1_28_ = self.conv40_1_28_(conv40_1_28_padding)
        batchnorm40_1_28_ = self.batchnorm40_1_28_(conv40_1_28_)
        relu40_1_28_ = self.relu40_1_28_(batchnorm40_1_28_)
        conv41_1_28_ = self.conv41_1_28_(relu40_1_28_)
        batchnorm41_1_28_ = self.batchnorm41_1_28_(conv41_1_28_)
        conv39_1_29_ = self.conv39_1_29_(relu37_)
        batchnorm39_1_29_ = self.batchnorm39_1_29_(conv39_1_29_)
        relu39_1_29_ = self.relu39_1_29_(batchnorm39_1_29_)
        conv40_1_29_padding = self.conv40_1_29_padding(relu39_1_29_)
        conv40_1_29_ = self.conv40_1_29_(conv40_1_29_padding)
        batchnorm40_1_29_ = self.batchnorm40_1_29_(conv40_1_29_)
        relu40_1_29_ = self.relu40_1_29_(batchnorm40_1_29_)
        conv41_1_29_ = self.conv41_1_29_(relu40_1_29_)
        batchnorm41_1_29_ = self.batchnorm41_1_29_(conv41_1_29_)
        conv39_1_30_ = self.conv39_1_30_(relu37_)
        batchnorm39_1_30_ = self.batchnorm39_1_30_(conv39_1_30_)
        relu39_1_30_ = self.relu39_1_30_(batchnorm39_1_30_)
        conv40_1_30_padding = self.conv40_1_30_padding(relu39_1_30_)
        conv40_1_30_ = self.conv40_1_30_(conv40_1_30_padding)
        batchnorm40_1_30_ = self.batchnorm40_1_30_(conv40_1_30_)
        relu40_1_30_ = self.relu40_1_30_(batchnorm40_1_30_)
        conv41_1_30_ = self.conv41_1_30_(relu40_1_30_)
        batchnorm41_1_30_ = self.batchnorm41_1_30_(conv41_1_30_)
        conv39_1_31_ = self.conv39_1_31_(relu37_)
        batchnorm39_1_31_ = self.batchnorm39_1_31_(conv39_1_31_)
        relu39_1_31_ = self.relu39_1_31_(batchnorm39_1_31_)
        conv40_1_31_padding = self.conv40_1_31_padding(relu39_1_31_)
        conv40_1_31_ = self.conv40_1_31_(conv40_1_31_padding)
        batchnorm40_1_31_ = self.batchnorm40_1_31_(conv40_1_31_)
        relu40_1_31_ = self.relu40_1_31_(batchnorm40_1_31_)
        conv41_1_31_ = self.conv41_1_31_(relu40_1_31_)
        batchnorm41_1_31_ = self.batchnorm41_1_31_(conv41_1_31_)
        conv39_1_32_ = self.conv39_1_32_(relu37_)
        batchnorm39_1_32_ = self.batchnorm39_1_32_(conv39_1_32_)
        relu39_1_32_ = self.relu39_1_32_(batchnorm39_1_32_)
        conv40_1_32_padding = self.conv40_1_32_padding(relu39_1_32_)
        conv40_1_32_ = self.conv40_1_32_(conv40_1_32_padding)
        batchnorm40_1_32_ = self.batchnorm40_1_32_(conv40_1_32_)
        relu40_1_32_ = self.relu40_1_32_(batchnorm40_1_32_)
        conv41_1_32_ = self.conv41_1_32_(relu40_1_32_)
        batchnorm41_1_32_ = self.batchnorm41_1_32_(conv41_1_32_)
        add42_1_ = batchnorm41_1_1_ + batchnorm41_1_2_ + batchnorm41_1_3_ + batchnorm41_1_4_ + batchnorm41_1_5_ + batchnorm41_1_6_ + batchnorm41_1_7_ + batchnorm41_1_8_ + batchnorm41_1_9_ + batchnorm41_1_10_ + batchnorm41_1_11_ + batchnorm41_1_12_ + batchnorm41_1_13_ + batchnorm41_1_14_ + batchnorm41_1_15_ + batchnorm41_1_16_ + batchnorm41_1_17_ + batchnorm41_1_18_ + batchnorm41_1_19_ + batchnorm41_1_20_ + batchnorm41_1_21_ + batchnorm41_1_22_ + batchnorm41_1_23_ + batchnorm41_1_24_ + batchnorm41_1_25_ + batchnorm41_1_26_ + batchnorm41_1_27_ + batchnorm41_1_28_ + batchnorm41_1_29_ + batchnorm41_1_30_ + batchnorm41_1_31_ + batchnorm41_1_32_
        add43_ = add42_1_ + relu37_
        relu43_ = self.relu43_(add43_)
        conv45_1_1_ = self.conv45_1_1_(relu43_)
        batchnorm45_1_1_ = self.batchnorm45_1_1_(conv45_1_1_)
        relu45_1_1_ = self.relu45_1_1_(batchnorm45_1_1_)
        conv46_1_1_padding = self.conv46_1_1_padding(relu45_1_1_)
        conv46_1_1_ = self.conv46_1_1_(conv46_1_1_padding)
        batchnorm46_1_1_ = self.batchnorm46_1_1_(conv46_1_1_)
        relu46_1_1_ = self.relu46_1_1_(batchnorm46_1_1_)
        conv47_1_1_ = self.conv47_1_1_(relu46_1_1_)
        batchnorm47_1_1_ = self.batchnorm47_1_1_(conv47_1_1_)
        conv45_1_2_ = self.conv45_1_2_(relu43_)
        batchnorm45_1_2_ = self.batchnorm45_1_2_(conv45_1_2_)
        relu45_1_2_ = self.relu45_1_2_(batchnorm45_1_2_)
        conv46_1_2_padding = self.conv46_1_2_padding(relu45_1_2_)
        conv46_1_2_ = self.conv46_1_2_(conv46_1_2_padding)
        batchnorm46_1_2_ = self.batchnorm46_1_2_(conv46_1_2_)
        relu46_1_2_ = self.relu46_1_2_(batchnorm46_1_2_)
        conv47_1_2_ = self.conv47_1_2_(relu46_1_2_)
        batchnorm47_1_2_ = self.batchnorm47_1_2_(conv47_1_2_)
        conv45_1_3_ = self.conv45_1_3_(relu43_)
        batchnorm45_1_3_ = self.batchnorm45_1_3_(conv45_1_3_)
        relu45_1_3_ = self.relu45_1_3_(batchnorm45_1_3_)
        conv46_1_3_padding = self.conv46_1_3_padding(relu45_1_3_)
        conv46_1_3_ = self.conv46_1_3_(conv46_1_3_padding)
        batchnorm46_1_3_ = self.batchnorm46_1_3_(conv46_1_3_)
        relu46_1_3_ = self.relu46_1_3_(batchnorm46_1_3_)
        conv47_1_3_ = self.conv47_1_3_(relu46_1_3_)
        batchnorm47_1_3_ = self.batchnorm47_1_3_(conv47_1_3_)
        conv45_1_4_ = self.conv45_1_4_(relu43_)
        batchnorm45_1_4_ = self.batchnorm45_1_4_(conv45_1_4_)
        relu45_1_4_ = self.relu45_1_4_(batchnorm45_1_4_)
        conv46_1_4_padding = self.conv46_1_4_padding(relu45_1_4_)
        conv46_1_4_ = self.conv46_1_4_(conv46_1_4_padding)
        batchnorm46_1_4_ = self.batchnorm46_1_4_(conv46_1_4_)
        relu46_1_4_ = self.relu46_1_4_(batchnorm46_1_4_)
        conv47_1_4_ = self.conv47_1_4_(relu46_1_4_)
        batchnorm47_1_4_ = self.batchnorm47_1_4_(conv47_1_4_)
        conv45_1_5_ = self.conv45_1_5_(relu43_)
        batchnorm45_1_5_ = self.batchnorm45_1_5_(conv45_1_5_)
        relu45_1_5_ = self.relu45_1_5_(batchnorm45_1_5_)
        conv46_1_5_padding = self.conv46_1_5_padding(relu45_1_5_)
        conv46_1_5_ = self.conv46_1_5_(conv46_1_5_padding)
        batchnorm46_1_5_ = self.batchnorm46_1_5_(conv46_1_5_)
        relu46_1_5_ = self.relu46_1_5_(batchnorm46_1_5_)
        conv47_1_5_ = self.conv47_1_5_(relu46_1_5_)
        batchnorm47_1_5_ = self.batchnorm47_1_5_(conv47_1_5_)
        conv45_1_6_ = self.conv45_1_6_(relu43_)
        batchnorm45_1_6_ = self.batchnorm45_1_6_(conv45_1_6_)
        relu45_1_6_ = self.relu45_1_6_(batchnorm45_1_6_)
        conv46_1_6_padding = self.conv46_1_6_padding(relu45_1_6_)
        conv46_1_6_ = self.conv46_1_6_(conv46_1_6_padding)
        batchnorm46_1_6_ = self.batchnorm46_1_6_(conv46_1_6_)
        relu46_1_6_ = self.relu46_1_6_(batchnorm46_1_6_)
        conv47_1_6_ = self.conv47_1_6_(relu46_1_6_)
        batchnorm47_1_6_ = self.batchnorm47_1_6_(conv47_1_6_)
        conv45_1_7_ = self.conv45_1_7_(relu43_)
        batchnorm45_1_7_ = self.batchnorm45_1_7_(conv45_1_7_)
        relu45_1_7_ = self.relu45_1_7_(batchnorm45_1_7_)
        conv46_1_7_padding = self.conv46_1_7_padding(relu45_1_7_)
        conv46_1_7_ = self.conv46_1_7_(conv46_1_7_padding)
        batchnorm46_1_7_ = self.batchnorm46_1_7_(conv46_1_7_)
        relu46_1_7_ = self.relu46_1_7_(batchnorm46_1_7_)
        conv47_1_7_ = self.conv47_1_7_(relu46_1_7_)
        batchnorm47_1_7_ = self.batchnorm47_1_7_(conv47_1_7_)
        conv45_1_8_ = self.conv45_1_8_(relu43_)
        batchnorm45_1_8_ = self.batchnorm45_1_8_(conv45_1_8_)
        relu45_1_8_ = self.relu45_1_8_(batchnorm45_1_8_)
        conv46_1_8_padding = self.conv46_1_8_padding(relu45_1_8_)
        conv46_1_8_ = self.conv46_1_8_(conv46_1_8_padding)
        batchnorm46_1_8_ = self.batchnorm46_1_8_(conv46_1_8_)
        relu46_1_8_ = self.relu46_1_8_(batchnorm46_1_8_)
        conv47_1_8_ = self.conv47_1_8_(relu46_1_8_)
        batchnorm47_1_8_ = self.batchnorm47_1_8_(conv47_1_8_)
        conv45_1_9_ = self.conv45_1_9_(relu43_)
        batchnorm45_1_9_ = self.batchnorm45_1_9_(conv45_1_9_)
        relu45_1_9_ = self.relu45_1_9_(batchnorm45_1_9_)
        conv46_1_9_padding = self.conv46_1_9_padding(relu45_1_9_)
        conv46_1_9_ = self.conv46_1_9_(conv46_1_9_padding)
        batchnorm46_1_9_ = self.batchnorm46_1_9_(conv46_1_9_)
        relu46_1_9_ = self.relu46_1_9_(batchnorm46_1_9_)
        conv47_1_9_ = self.conv47_1_9_(relu46_1_9_)
        batchnorm47_1_9_ = self.batchnorm47_1_9_(conv47_1_9_)
        conv45_1_10_ = self.conv45_1_10_(relu43_)
        batchnorm45_1_10_ = self.batchnorm45_1_10_(conv45_1_10_)
        relu45_1_10_ = self.relu45_1_10_(batchnorm45_1_10_)
        conv46_1_10_padding = self.conv46_1_10_padding(relu45_1_10_)
        conv46_1_10_ = self.conv46_1_10_(conv46_1_10_padding)
        batchnorm46_1_10_ = self.batchnorm46_1_10_(conv46_1_10_)
        relu46_1_10_ = self.relu46_1_10_(batchnorm46_1_10_)
        conv47_1_10_ = self.conv47_1_10_(relu46_1_10_)
        batchnorm47_1_10_ = self.batchnorm47_1_10_(conv47_1_10_)
        conv45_1_11_ = self.conv45_1_11_(relu43_)
        batchnorm45_1_11_ = self.batchnorm45_1_11_(conv45_1_11_)
        relu45_1_11_ = self.relu45_1_11_(batchnorm45_1_11_)
        conv46_1_11_padding = self.conv46_1_11_padding(relu45_1_11_)
        conv46_1_11_ = self.conv46_1_11_(conv46_1_11_padding)
        batchnorm46_1_11_ = self.batchnorm46_1_11_(conv46_1_11_)
        relu46_1_11_ = self.relu46_1_11_(batchnorm46_1_11_)
        conv47_1_11_ = self.conv47_1_11_(relu46_1_11_)
        batchnorm47_1_11_ = self.batchnorm47_1_11_(conv47_1_11_)
        conv45_1_12_ = self.conv45_1_12_(relu43_)
        batchnorm45_1_12_ = self.batchnorm45_1_12_(conv45_1_12_)
        relu45_1_12_ = self.relu45_1_12_(batchnorm45_1_12_)
        conv46_1_12_padding = self.conv46_1_12_padding(relu45_1_12_)
        conv46_1_12_ = self.conv46_1_12_(conv46_1_12_padding)
        batchnorm46_1_12_ = self.batchnorm46_1_12_(conv46_1_12_)
        relu46_1_12_ = self.relu46_1_12_(batchnorm46_1_12_)
        conv47_1_12_ = self.conv47_1_12_(relu46_1_12_)
        batchnorm47_1_12_ = self.batchnorm47_1_12_(conv47_1_12_)
        conv45_1_13_ = self.conv45_1_13_(relu43_)
        batchnorm45_1_13_ = self.batchnorm45_1_13_(conv45_1_13_)
        relu45_1_13_ = self.relu45_1_13_(batchnorm45_1_13_)
        conv46_1_13_padding = self.conv46_1_13_padding(relu45_1_13_)
        conv46_1_13_ = self.conv46_1_13_(conv46_1_13_padding)
        batchnorm46_1_13_ = self.batchnorm46_1_13_(conv46_1_13_)
        relu46_1_13_ = self.relu46_1_13_(batchnorm46_1_13_)
        conv47_1_13_ = self.conv47_1_13_(relu46_1_13_)
        batchnorm47_1_13_ = self.batchnorm47_1_13_(conv47_1_13_)
        conv45_1_14_ = self.conv45_1_14_(relu43_)
        batchnorm45_1_14_ = self.batchnorm45_1_14_(conv45_1_14_)
        relu45_1_14_ = self.relu45_1_14_(batchnorm45_1_14_)
        conv46_1_14_padding = self.conv46_1_14_padding(relu45_1_14_)
        conv46_1_14_ = self.conv46_1_14_(conv46_1_14_padding)
        batchnorm46_1_14_ = self.batchnorm46_1_14_(conv46_1_14_)
        relu46_1_14_ = self.relu46_1_14_(batchnorm46_1_14_)
        conv47_1_14_ = self.conv47_1_14_(relu46_1_14_)
        batchnorm47_1_14_ = self.batchnorm47_1_14_(conv47_1_14_)
        conv45_1_15_ = self.conv45_1_15_(relu43_)
        batchnorm45_1_15_ = self.batchnorm45_1_15_(conv45_1_15_)
        relu45_1_15_ = self.relu45_1_15_(batchnorm45_1_15_)
        conv46_1_15_padding = self.conv46_1_15_padding(relu45_1_15_)
        conv46_1_15_ = self.conv46_1_15_(conv46_1_15_padding)
        batchnorm46_1_15_ = self.batchnorm46_1_15_(conv46_1_15_)
        relu46_1_15_ = self.relu46_1_15_(batchnorm46_1_15_)
        conv47_1_15_ = self.conv47_1_15_(relu46_1_15_)
        batchnorm47_1_15_ = self.batchnorm47_1_15_(conv47_1_15_)
        conv45_1_16_ = self.conv45_1_16_(relu43_)
        batchnorm45_1_16_ = self.batchnorm45_1_16_(conv45_1_16_)
        relu45_1_16_ = self.relu45_1_16_(batchnorm45_1_16_)
        conv46_1_16_padding = self.conv46_1_16_padding(relu45_1_16_)
        conv46_1_16_ = self.conv46_1_16_(conv46_1_16_padding)
        batchnorm46_1_16_ = self.batchnorm46_1_16_(conv46_1_16_)
        relu46_1_16_ = self.relu46_1_16_(batchnorm46_1_16_)
        conv47_1_16_ = self.conv47_1_16_(relu46_1_16_)
        batchnorm47_1_16_ = self.batchnorm47_1_16_(conv47_1_16_)
        conv45_1_17_ = self.conv45_1_17_(relu43_)
        batchnorm45_1_17_ = self.batchnorm45_1_17_(conv45_1_17_)
        relu45_1_17_ = self.relu45_1_17_(batchnorm45_1_17_)
        conv46_1_17_padding = self.conv46_1_17_padding(relu45_1_17_)
        conv46_1_17_ = self.conv46_1_17_(conv46_1_17_padding)
        batchnorm46_1_17_ = self.batchnorm46_1_17_(conv46_1_17_)
        relu46_1_17_ = self.relu46_1_17_(batchnorm46_1_17_)
        conv47_1_17_ = self.conv47_1_17_(relu46_1_17_)
        batchnorm47_1_17_ = self.batchnorm47_1_17_(conv47_1_17_)
        conv45_1_18_ = self.conv45_1_18_(relu43_)
        batchnorm45_1_18_ = self.batchnorm45_1_18_(conv45_1_18_)
        relu45_1_18_ = self.relu45_1_18_(batchnorm45_1_18_)
        conv46_1_18_padding = self.conv46_1_18_padding(relu45_1_18_)
        conv46_1_18_ = self.conv46_1_18_(conv46_1_18_padding)
        batchnorm46_1_18_ = self.batchnorm46_1_18_(conv46_1_18_)
        relu46_1_18_ = self.relu46_1_18_(batchnorm46_1_18_)
        conv47_1_18_ = self.conv47_1_18_(relu46_1_18_)
        batchnorm47_1_18_ = self.batchnorm47_1_18_(conv47_1_18_)
        conv45_1_19_ = self.conv45_1_19_(relu43_)
        batchnorm45_1_19_ = self.batchnorm45_1_19_(conv45_1_19_)
        relu45_1_19_ = self.relu45_1_19_(batchnorm45_1_19_)
        conv46_1_19_padding = self.conv46_1_19_padding(relu45_1_19_)
        conv46_1_19_ = self.conv46_1_19_(conv46_1_19_padding)
        batchnorm46_1_19_ = self.batchnorm46_1_19_(conv46_1_19_)
        relu46_1_19_ = self.relu46_1_19_(batchnorm46_1_19_)
        conv47_1_19_ = self.conv47_1_19_(relu46_1_19_)
        batchnorm47_1_19_ = self.batchnorm47_1_19_(conv47_1_19_)
        conv45_1_20_ = self.conv45_1_20_(relu43_)
        batchnorm45_1_20_ = self.batchnorm45_1_20_(conv45_1_20_)
        relu45_1_20_ = self.relu45_1_20_(batchnorm45_1_20_)
        conv46_1_20_padding = self.conv46_1_20_padding(relu45_1_20_)
        conv46_1_20_ = self.conv46_1_20_(conv46_1_20_padding)
        batchnorm46_1_20_ = self.batchnorm46_1_20_(conv46_1_20_)
        relu46_1_20_ = self.relu46_1_20_(batchnorm46_1_20_)
        conv47_1_20_ = self.conv47_1_20_(relu46_1_20_)
        batchnorm47_1_20_ = self.batchnorm47_1_20_(conv47_1_20_)
        conv45_1_21_ = self.conv45_1_21_(relu43_)
        batchnorm45_1_21_ = self.batchnorm45_1_21_(conv45_1_21_)
        relu45_1_21_ = self.relu45_1_21_(batchnorm45_1_21_)
        conv46_1_21_padding = self.conv46_1_21_padding(relu45_1_21_)
        conv46_1_21_ = self.conv46_1_21_(conv46_1_21_padding)
        batchnorm46_1_21_ = self.batchnorm46_1_21_(conv46_1_21_)
        relu46_1_21_ = self.relu46_1_21_(batchnorm46_1_21_)
        conv47_1_21_ = self.conv47_1_21_(relu46_1_21_)
        batchnorm47_1_21_ = self.batchnorm47_1_21_(conv47_1_21_)
        conv45_1_22_ = self.conv45_1_22_(relu43_)
        batchnorm45_1_22_ = self.batchnorm45_1_22_(conv45_1_22_)
        relu45_1_22_ = self.relu45_1_22_(batchnorm45_1_22_)
        conv46_1_22_padding = self.conv46_1_22_padding(relu45_1_22_)
        conv46_1_22_ = self.conv46_1_22_(conv46_1_22_padding)
        batchnorm46_1_22_ = self.batchnorm46_1_22_(conv46_1_22_)
        relu46_1_22_ = self.relu46_1_22_(batchnorm46_1_22_)
        conv47_1_22_ = self.conv47_1_22_(relu46_1_22_)
        batchnorm47_1_22_ = self.batchnorm47_1_22_(conv47_1_22_)
        conv45_1_23_ = self.conv45_1_23_(relu43_)
        batchnorm45_1_23_ = self.batchnorm45_1_23_(conv45_1_23_)
        relu45_1_23_ = self.relu45_1_23_(batchnorm45_1_23_)
        conv46_1_23_padding = self.conv46_1_23_padding(relu45_1_23_)
        conv46_1_23_ = self.conv46_1_23_(conv46_1_23_padding)
        batchnorm46_1_23_ = self.batchnorm46_1_23_(conv46_1_23_)
        relu46_1_23_ = self.relu46_1_23_(batchnorm46_1_23_)
        conv47_1_23_ = self.conv47_1_23_(relu46_1_23_)
        batchnorm47_1_23_ = self.batchnorm47_1_23_(conv47_1_23_)
        conv45_1_24_ = self.conv45_1_24_(relu43_)
        batchnorm45_1_24_ = self.batchnorm45_1_24_(conv45_1_24_)
        relu45_1_24_ = self.relu45_1_24_(batchnorm45_1_24_)
        conv46_1_24_padding = self.conv46_1_24_padding(relu45_1_24_)
        conv46_1_24_ = self.conv46_1_24_(conv46_1_24_padding)
        batchnorm46_1_24_ = self.batchnorm46_1_24_(conv46_1_24_)
        relu46_1_24_ = self.relu46_1_24_(batchnorm46_1_24_)
        conv47_1_24_ = self.conv47_1_24_(relu46_1_24_)
        batchnorm47_1_24_ = self.batchnorm47_1_24_(conv47_1_24_)
        conv45_1_25_ = self.conv45_1_25_(relu43_)
        batchnorm45_1_25_ = self.batchnorm45_1_25_(conv45_1_25_)
        relu45_1_25_ = self.relu45_1_25_(batchnorm45_1_25_)
        conv46_1_25_padding = self.conv46_1_25_padding(relu45_1_25_)
        conv46_1_25_ = self.conv46_1_25_(conv46_1_25_padding)
        batchnorm46_1_25_ = self.batchnorm46_1_25_(conv46_1_25_)
        relu46_1_25_ = self.relu46_1_25_(batchnorm46_1_25_)
        conv47_1_25_ = self.conv47_1_25_(relu46_1_25_)
        batchnorm47_1_25_ = self.batchnorm47_1_25_(conv47_1_25_)
        conv45_1_26_ = self.conv45_1_26_(relu43_)
        batchnorm45_1_26_ = self.batchnorm45_1_26_(conv45_1_26_)
        relu45_1_26_ = self.relu45_1_26_(batchnorm45_1_26_)
        conv46_1_26_padding = self.conv46_1_26_padding(relu45_1_26_)
        conv46_1_26_ = self.conv46_1_26_(conv46_1_26_padding)
        batchnorm46_1_26_ = self.batchnorm46_1_26_(conv46_1_26_)
        relu46_1_26_ = self.relu46_1_26_(batchnorm46_1_26_)
        conv47_1_26_ = self.conv47_1_26_(relu46_1_26_)
        batchnorm47_1_26_ = self.batchnorm47_1_26_(conv47_1_26_)
        conv45_1_27_ = self.conv45_1_27_(relu43_)
        batchnorm45_1_27_ = self.batchnorm45_1_27_(conv45_1_27_)
        relu45_1_27_ = self.relu45_1_27_(batchnorm45_1_27_)
        conv46_1_27_padding = self.conv46_1_27_padding(relu45_1_27_)
        conv46_1_27_ = self.conv46_1_27_(conv46_1_27_padding)
        batchnorm46_1_27_ = self.batchnorm46_1_27_(conv46_1_27_)
        relu46_1_27_ = self.relu46_1_27_(batchnorm46_1_27_)
        conv47_1_27_ = self.conv47_1_27_(relu46_1_27_)
        batchnorm47_1_27_ = self.batchnorm47_1_27_(conv47_1_27_)
        conv45_1_28_ = self.conv45_1_28_(relu43_)
        batchnorm45_1_28_ = self.batchnorm45_1_28_(conv45_1_28_)
        relu45_1_28_ = self.relu45_1_28_(batchnorm45_1_28_)
        conv46_1_28_padding = self.conv46_1_28_padding(relu45_1_28_)
        conv46_1_28_ = self.conv46_1_28_(conv46_1_28_padding)
        batchnorm46_1_28_ = self.batchnorm46_1_28_(conv46_1_28_)
        relu46_1_28_ = self.relu46_1_28_(batchnorm46_1_28_)
        conv47_1_28_ = self.conv47_1_28_(relu46_1_28_)
        batchnorm47_1_28_ = self.batchnorm47_1_28_(conv47_1_28_)
        conv45_1_29_ = self.conv45_1_29_(relu43_)
        batchnorm45_1_29_ = self.batchnorm45_1_29_(conv45_1_29_)
        relu45_1_29_ = self.relu45_1_29_(batchnorm45_1_29_)
        conv46_1_29_padding = self.conv46_1_29_padding(relu45_1_29_)
        conv46_1_29_ = self.conv46_1_29_(conv46_1_29_padding)
        batchnorm46_1_29_ = self.batchnorm46_1_29_(conv46_1_29_)
        relu46_1_29_ = self.relu46_1_29_(batchnorm46_1_29_)
        conv47_1_29_ = self.conv47_1_29_(relu46_1_29_)
        batchnorm47_1_29_ = self.batchnorm47_1_29_(conv47_1_29_)
        conv45_1_30_ = self.conv45_1_30_(relu43_)
        batchnorm45_1_30_ = self.batchnorm45_1_30_(conv45_1_30_)
        relu45_1_30_ = self.relu45_1_30_(batchnorm45_1_30_)
        conv46_1_30_padding = self.conv46_1_30_padding(relu45_1_30_)
        conv46_1_30_ = self.conv46_1_30_(conv46_1_30_padding)
        batchnorm46_1_30_ = self.batchnorm46_1_30_(conv46_1_30_)
        relu46_1_30_ = self.relu46_1_30_(batchnorm46_1_30_)
        conv47_1_30_ = self.conv47_1_30_(relu46_1_30_)
        batchnorm47_1_30_ = self.batchnorm47_1_30_(conv47_1_30_)
        conv45_1_31_ = self.conv45_1_31_(relu43_)
        batchnorm45_1_31_ = self.batchnorm45_1_31_(conv45_1_31_)
        relu45_1_31_ = self.relu45_1_31_(batchnorm45_1_31_)
        conv46_1_31_padding = self.conv46_1_31_padding(relu45_1_31_)
        conv46_1_31_ = self.conv46_1_31_(conv46_1_31_padding)
        batchnorm46_1_31_ = self.batchnorm46_1_31_(conv46_1_31_)
        relu46_1_31_ = self.relu46_1_31_(batchnorm46_1_31_)
        conv47_1_31_ = self.conv47_1_31_(relu46_1_31_)
        batchnorm47_1_31_ = self.batchnorm47_1_31_(conv47_1_31_)
        conv45_1_32_ = self.conv45_1_32_(relu43_)
        batchnorm45_1_32_ = self.batchnorm45_1_32_(conv45_1_32_)
        relu45_1_32_ = self.relu45_1_32_(batchnorm45_1_32_)
        conv46_1_32_padding = self.conv46_1_32_padding(relu45_1_32_)
        conv46_1_32_ = self.conv46_1_32_(conv46_1_32_padding)
        batchnorm46_1_32_ = self.batchnorm46_1_32_(conv46_1_32_)
        relu46_1_32_ = self.relu46_1_32_(batchnorm46_1_32_)
        conv47_1_32_ = self.conv47_1_32_(relu46_1_32_)
        batchnorm47_1_32_ = self.batchnorm47_1_32_(conv47_1_32_)
        add48_1_ = batchnorm47_1_1_ + batchnorm47_1_2_ + batchnorm47_1_3_ + batchnorm47_1_4_ + batchnorm47_1_5_ + batchnorm47_1_6_ + batchnorm47_1_7_ + batchnorm47_1_8_ + batchnorm47_1_9_ + batchnorm47_1_10_ + batchnorm47_1_11_ + batchnorm47_1_12_ + batchnorm47_1_13_ + batchnorm47_1_14_ + batchnorm47_1_15_ + batchnorm47_1_16_ + batchnorm47_1_17_ + batchnorm47_1_18_ + batchnorm47_1_19_ + batchnorm47_1_20_ + batchnorm47_1_21_ + batchnorm47_1_22_ + batchnorm47_1_23_ + batchnorm47_1_24_ + batchnorm47_1_25_ + batchnorm47_1_26_ + batchnorm47_1_27_ + batchnorm47_1_28_ + batchnorm47_1_29_ + batchnorm47_1_30_ + batchnorm47_1_31_ + batchnorm47_1_32_
        conv44_2_ = self.conv44_2_(relu43_)
        batchnorm44_2_ = self.batchnorm44_2_(conv44_2_)
        add49_ = add48_1_ + batchnorm44_2_
        relu49_ = self.relu49_(add49_)
        conv51_1_1_ = self.conv51_1_1_(relu49_)
        batchnorm51_1_1_ = self.batchnorm51_1_1_(conv51_1_1_)
        relu51_1_1_ = self.relu51_1_1_(batchnorm51_1_1_)
        conv52_1_1_padding = self.conv52_1_1_padding(relu51_1_1_)
        conv52_1_1_ = self.conv52_1_1_(conv52_1_1_padding)
        batchnorm52_1_1_ = self.batchnorm52_1_1_(conv52_1_1_)
        relu52_1_1_ = self.relu52_1_1_(batchnorm52_1_1_)
        conv53_1_1_ = self.conv53_1_1_(relu52_1_1_)
        batchnorm53_1_1_ = self.batchnorm53_1_1_(conv53_1_1_)
        conv51_1_2_ = self.conv51_1_2_(relu49_)
        batchnorm51_1_2_ = self.batchnorm51_1_2_(conv51_1_2_)
        relu51_1_2_ = self.relu51_1_2_(batchnorm51_1_2_)
        conv52_1_2_padding = self.conv52_1_2_padding(relu51_1_2_)
        conv52_1_2_ = self.conv52_1_2_(conv52_1_2_padding)
        batchnorm52_1_2_ = self.batchnorm52_1_2_(conv52_1_2_)
        relu52_1_2_ = self.relu52_1_2_(batchnorm52_1_2_)
        conv53_1_2_ = self.conv53_1_2_(relu52_1_2_)
        batchnorm53_1_2_ = self.batchnorm53_1_2_(conv53_1_2_)
        conv51_1_3_ = self.conv51_1_3_(relu49_)
        batchnorm51_1_3_ = self.batchnorm51_1_3_(conv51_1_3_)
        relu51_1_3_ = self.relu51_1_3_(batchnorm51_1_3_)
        conv52_1_3_padding = self.conv52_1_3_padding(relu51_1_3_)
        conv52_1_3_ = self.conv52_1_3_(conv52_1_3_padding)
        batchnorm52_1_3_ = self.batchnorm52_1_3_(conv52_1_3_)
        relu52_1_3_ = self.relu52_1_3_(batchnorm52_1_3_)
        conv53_1_3_ = self.conv53_1_3_(relu52_1_3_)
        batchnorm53_1_3_ = self.batchnorm53_1_3_(conv53_1_3_)
        conv51_1_4_ = self.conv51_1_4_(relu49_)
        batchnorm51_1_4_ = self.batchnorm51_1_4_(conv51_1_4_)
        relu51_1_4_ = self.relu51_1_4_(batchnorm51_1_4_)
        conv52_1_4_padding = self.conv52_1_4_padding(relu51_1_4_)
        conv52_1_4_ = self.conv52_1_4_(conv52_1_4_padding)
        batchnorm52_1_4_ = self.batchnorm52_1_4_(conv52_1_4_)
        relu52_1_4_ = self.relu52_1_4_(batchnorm52_1_4_)
        conv53_1_4_ = self.conv53_1_4_(relu52_1_4_)
        batchnorm53_1_4_ = self.batchnorm53_1_4_(conv53_1_4_)
        conv51_1_5_ = self.conv51_1_5_(relu49_)
        batchnorm51_1_5_ = self.batchnorm51_1_5_(conv51_1_5_)
        relu51_1_5_ = self.relu51_1_5_(batchnorm51_1_5_)
        conv52_1_5_padding = self.conv52_1_5_padding(relu51_1_5_)
        conv52_1_5_ = self.conv52_1_5_(conv52_1_5_padding)
        batchnorm52_1_5_ = self.batchnorm52_1_5_(conv52_1_5_)
        relu52_1_5_ = self.relu52_1_5_(batchnorm52_1_5_)
        conv53_1_5_ = self.conv53_1_5_(relu52_1_5_)
        batchnorm53_1_5_ = self.batchnorm53_1_5_(conv53_1_5_)
        conv51_1_6_ = self.conv51_1_6_(relu49_)
        batchnorm51_1_6_ = self.batchnorm51_1_6_(conv51_1_6_)
        relu51_1_6_ = self.relu51_1_6_(batchnorm51_1_6_)
        conv52_1_6_padding = self.conv52_1_6_padding(relu51_1_6_)
        conv52_1_6_ = self.conv52_1_6_(conv52_1_6_padding)
        batchnorm52_1_6_ = self.batchnorm52_1_6_(conv52_1_6_)
        relu52_1_6_ = self.relu52_1_6_(batchnorm52_1_6_)
        conv53_1_6_ = self.conv53_1_6_(relu52_1_6_)
        batchnorm53_1_6_ = self.batchnorm53_1_6_(conv53_1_6_)
        conv51_1_7_ = self.conv51_1_7_(relu49_)
        batchnorm51_1_7_ = self.batchnorm51_1_7_(conv51_1_7_)
        relu51_1_7_ = self.relu51_1_7_(batchnorm51_1_7_)
        conv52_1_7_padding = self.conv52_1_7_padding(relu51_1_7_)
        conv52_1_7_ = self.conv52_1_7_(conv52_1_7_padding)
        batchnorm52_1_7_ = self.batchnorm52_1_7_(conv52_1_7_)
        relu52_1_7_ = self.relu52_1_7_(batchnorm52_1_7_)
        conv53_1_7_ = self.conv53_1_7_(relu52_1_7_)
        batchnorm53_1_7_ = self.batchnorm53_1_7_(conv53_1_7_)
        conv51_1_8_ = self.conv51_1_8_(relu49_)
        batchnorm51_1_8_ = self.batchnorm51_1_8_(conv51_1_8_)
        relu51_1_8_ = self.relu51_1_8_(batchnorm51_1_8_)
        conv52_1_8_padding = self.conv52_1_8_padding(relu51_1_8_)
        conv52_1_8_ = self.conv52_1_8_(conv52_1_8_padding)
        batchnorm52_1_8_ = self.batchnorm52_1_8_(conv52_1_8_)
        relu52_1_8_ = self.relu52_1_8_(batchnorm52_1_8_)
        conv53_1_8_ = self.conv53_1_8_(relu52_1_8_)
        batchnorm53_1_8_ = self.batchnorm53_1_8_(conv53_1_8_)
        conv51_1_9_ = self.conv51_1_9_(relu49_)
        batchnorm51_1_9_ = self.batchnorm51_1_9_(conv51_1_9_)
        relu51_1_9_ = self.relu51_1_9_(batchnorm51_1_9_)
        conv52_1_9_padding = self.conv52_1_9_padding(relu51_1_9_)
        conv52_1_9_ = self.conv52_1_9_(conv52_1_9_padding)
        batchnorm52_1_9_ = self.batchnorm52_1_9_(conv52_1_9_)
        relu52_1_9_ = self.relu52_1_9_(batchnorm52_1_9_)
        conv53_1_9_ = self.conv53_1_9_(relu52_1_9_)
        batchnorm53_1_9_ = self.batchnorm53_1_9_(conv53_1_9_)
        conv51_1_10_ = self.conv51_1_10_(relu49_)
        batchnorm51_1_10_ = self.batchnorm51_1_10_(conv51_1_10_)
        relu51_1_10_ = self.relu51_1_10_(batchnorm51_1_10_)
        conv52_1_10_padding = self.conv52_1_10_padding(relu51_1_10_)
        conv52_1_10_ = self.conv52_1_10_(conv52_1_10_padding)
        batchnorm52_1_10_ = self.batchnorm52_1_10_(conv52_1_10_)
        relu52_1_10_ = self.relu52_1_10_(batchnorm52_1_10_)
        conv53_1_10_ = self.conv53_1_10_(relu52_1_10_)
        batchnorm53_1_10_ = self.batchnorm53_1_10_(conv53_1_10_)
        conv51_1_11_ = self.conv51_1_11_(relu49_)
        batchnorm51_1_11_ = self.batchnorm51_1_11_(conv51_1_11_)
        relu51_1_11_ = self.relu51_1_11_(batchnorm51_1_11_)
        conv52_1_11_padding = self.conv52_1_11_padding(relu51_1_11_)
        conv52_1_11_ = self.conv52_1_11_(conv52_1_11_padding)
        batchnorm52_1_11_ = self.batchnorm52_1_11_(conv52_1_11_)
        relu52_1_11_ = self.relu52_1_11_(batchnorm52_1_11_)
        conv53_1_11_ = self.conv53_1_11_(relu52_1_11_)
        batchnorm53_1_11_ = self.batchnorm53_1_11_(conv53_1_11_)
        conv51_1_12_ = self.conv51_1_12_(relu49_)
        batchnorm51_1_12_ = self.batchnorm51_1_12_(conv51_1_12_)
        relu51_1_12_ = self.relu51_1_12_(batchnorm51_1_12_)
        conv52_1_12_padding = self.conv52_1_12_padding(relu51_1_12_)
        conv52_1_12_ = self.conv52_1_12_(conv52_1_12_padding)
        batchnorm52_1_12_ = self.batchnorm52_1_12_(conv52_1_12_)
        relu52_1_12_ = self.relu52_1_12_(batchnorm52_1_12_)
        conv53_1_12_ = self.conv53_1_12_(relu52_1_12_)
        batchnorm53_1_12_ = self.batchnorm53_1_12_(conv53_1_12_)
        conv51_1_13_ = self.conv51_1_13_(relu49_)
        batchnorm51_1_13_ = self.batchnorm51_1_13_(conv51_1_13_)
        relu51_1_13_ = self.relu51_1_13_(batchnorm51_1_13_)
        conv52_1_13_padding = self.conv52_1_13_padding(relu51_1_13_)
        conv52_1_13_ = self.conv52_1_13_(conv52_1_13_padding)
        batchnorm52_1_13_ = self.batchnorm52_1_13_(conv52_1_13_)
        relu52_1_13_ = self.relu52_1_13_(batchnorm52_1_13_)
        conv53_1_13_ = self.conv53_1_13_(relu52_1_13_)
        batchnorm53_1_13_ = self.batchnorm53_1_13_(conv53_1_13_)
        conv51_1_14_ = self.conv51_1_14_(relu49_)
        batchnorm51_1_14_ = self.batchnorm51_1_14_(conv51_1_14_)
        relu51_1_14_ = self.relu51_1_14_(batchnorm51_1_14_)
        conv52_1_14_padding = self.conv52_1_14_padding(relu51_1_14_)
        conv52_1_14_ = self.conv52_1_14_(conv52_1_14_padding)
        batchnorm52_1_14_ = self.batchnorm52_1_14_(conv52_1_14_)
        relu52_1_14_ = self.relu52_1_14_(batchnorm52_1_14_)
        conv53_1_14_ = self.conv53_1_14_(relu52_1_14_)
        batchnorm53_1_14_ = self.batchnorm53_1_14_(conv53_1_14_)
        conv51_1_15_ = self.conv51_1_15_(relu49_)
        batchnorm51_1_15_ = self.batchnorm51_1_15_(conv51_1_15_)
        relu51_1_15_ = self.relu51_1_15_(batchnorm51_1_15_)
        conv52_1_15_padding = self.conv52_1_15_padding(relu51_1_15_)
        conv52_1_15_ = self.conv52_1_15_(conv52_1_15_padding)
        batchnorm52_1_15_ = self.batchnorm52_1_15_(conv52_1_15_)
        relu52_1_15_ = self.relu52_1_15_(batchnorm52_1_15_)
        conv53_1_15_ = self.conv53_1_15_(relu52_1_15_)
        batchnorm53_1_15_ = self.batchnorm53_1_15_(conv53_1_15_)
        conv51_1_16_ = self.conv51_1_16_(relu49_)
        batchnorm51_1_16_ = self.batchnorm51_1_16_(conv51_1_16_)
        relu51_1_16_ = self.relu51_1_16_(batchnorm51_1_16_)
        conv52_1_16_padding = self.conv52_1_16_padding(relu51_1_16_)
        conv52_1_16_ = self.conv52_1_16_(conv52_1_16_padding)
        batchnorm52_1_16_ = self.batchnorm52_1_16_(conv52_1_16_)
        relu52_1_16_ = self.relu52_1_16_(batchnorm52_1_16_)
        conv53_1_16_ = self.conv53_1_16_(relu52_1_16_)
        batchnorm53_1_16_ = self.batchnorm53_1_16_(conv53_1_16_)
        conv51_1_17_ = self.conv51_1_17_(relu49_)
        batchnorm51_1_17_ = self.batchnorm51_1_17_(conv51_1_17_)
        relu51_1_17_ = self.relu51_1_17_(batchnorm51_1_17_)
        conv52_1_17_padding = self.conv52_1_17_padding(relu51_1_17_)
        conv52_1_17_ = self.conv52_1_17_(conv52_1_17_padding)
        batchnorm52_1_17_ = self.batchnorm52_1_17_(conv52_1_17_)
        relu52_1_17_ = self.relu52_1_17_(batchnorm52_1_17_)
        conv53_1_17_ = self.conv53_1_17_(relu52_1_17_)
        batchnorm53_1_17_ = self.batchnorm53_1_17_(conv53_1_17_)
        conv51_1_18_ = self.conv51_1_18_(relu49_)
        batchnorm51_1_18_ = self.batchnorm51_1_18_(conv51_1_18_)
        relu51_1_18_ = self.relu51_1_18_(batchnorm51_1_18_)
        conv52_1_18_padding = self.conv52_1_18_padding(relu51_1_18_)
        conv52_1_18_ = self.conv52_1_18_(conv52_1_18_padding)
        batchnorm52_1_18_ = self.batchnorm52_1_18_(conv52_1_18_)
        relu52_1_18_ = self.relu52_1_18_(batchnorm52_1_18_)
        conv53_1_18_ = self.conv53_1_18_(relu52_1_18_)
        batchnorm53_1_18_ = self.batchnorm53_1_18_(conv53_1_18_)
        conv51_1_19_ = self.conv51_1_19_(relu49_)
        batchnorm51_1_19_ = self.batchnorm51_1_19_(conv51_1_19_)
        relu51_1_19_ = self.relu51_1_19_(batchnorm51_1_19_)
        conv52_1_19_padding = self.conv52_1_19_padding(relu51_1_19_)
        conv52_1_19_ = self.conv52_1_19_(conv52_1_19_padding)
        batchnorm52_1_19_ = self.batchnorm52_1_19_(conv52_1_19_)
        relu52_1_19_ = self.relu52_1_19_(batchnorm52_1_19_)
        conv53_1_19_ = self.conv53_1_19_(relu52_1_19_)
        batchnorm53_1_19_ = self.batchnorm53_1_19_(conv53_1_19_)
        conv51_1_20_ = self.conv51_1_20_(relu49_)
        batchnorm51_1_20_ = self.batchnorm51_1_20_(conv51_1_20_)
        relu51_1_20_ = self.relu51_1_20_(batchnorm51_1_20_)
        conv52_1_20_padding = self.conv52_1_20_padding(relu51_1_20_)
        conv52_1_20_ = self.conv52_1_20_(conv52_1_20_padding)
        batchnorm52_1_20_ = self.batchnorm52_1_20_(conv52_1_20_)
        relu52_1_20_ = self.relu52_1_20_(batchnorm52_1_20_)
        conv53_1_20_ = self.conv53_1_20_(relu52_1_20_)
        batchnorm53_1_20_ = self.batchnorm53_1_20_(conv53_1_20_)
        conv51_1_21_ = self.conv51_1_21_(relu49_)
        batchnorm51_1_21_ = self.batchnorm51_1_21_(conv51_1_21_)
        relu51_1_21_ = self.relu51_1_21_(batchnorm51_1_21_)
        conv52_1_21_padding = self.conv52_1_21_padding(relu51_1_21_)
        conv52_1_21_ = self.conv52_1_21_(conv52_1_21_padding)
        batchnorm52_1_21_ = self.batchnorm52_1_21_(conv52_1_21_)
        relu52_1_21_ = self.relu52_1_21_(batchnorm52_1_21_)
        conv53_1_21_ = self.conv53_1_21_(relu52_1_21_)
        batchnorm53_1_21_ = self.batchnorm53_1_21_(conv53_1_21_)
        conv51_1_22_ = self.conv51_1_22_(relu49_)
        batchnorm51_1_22_ = self.batchnorm51_1_22_(conv51_1_22_)
        relu51_1_22_ = self.relu51_1_22_(batchnorm51_1_22_)
        conv52_1_22_padding = self.conv52_1_22_padding(relu51_1_22_)
        conv52_1_22_ = self.conv52_1_22_(conv52_1_22_padding)
        batchnorm52_1_22_ = self.batchnorm52_1_22_(conv52_1_22_)
        relu52_1_22_ = self.relu52_1_22_(batchnorm52_1_22_)
        conv53_1_22_ = self.conv53_1_22_(relu52_1_22_)
        batchnorm53_1_22_ = self.batchnorm53_1_22_(conv53_1_22_)
        conv51_1_23_ = self.conv51_1_23_(relu49_)
        batchnorm51_1_23_ = self.batchnorm51_1_23_(conv51_1_23_)
        relu51_1_23_ = self.relu51_1_23_(batchnorm51_1_23_)
        conv52_1_23_padding = self.conv52_1_23_padding(relu51_1_23_)
        conv52_1_23_ = self.conv52_1_23_(conv52_1_23_padding)
        batchnorm52_1_23_ = self.batchnorm52_1_23_(conv52_1_23_)
        relu52_1_23_ = self.relu52_1_23_(batchnorm52_1_23_)
        conv53_1_23_ = self.conv53_1_23_(relu52_1_23_)
        batchnorm53_1_23_ = self.batchnorm53_1_23_(conv53_1_23_)
        conv51_1_24_ = self.conv51_1_24_(relu49_)
        batchnorm51_1_24_ = self.batchnorm51_1_24_(conv51_1_24_)
        relu51_1_24_ = self.relu51_1_24_(batchnorm51_1_24_)
        conv52_1_24_padding = self.conv52_1_24_padding(relu51_1_24_)
        conv52_1_24_ = self.conv52_1_24_(conv52_1_24_padding)
        batchnorm52_1_24_ = self.batchnorm52_1_24_(conv52_1_24_)
        relu52_1_24_ = self.relu52_1_24_(batchnorm52_1_24_)
        conv53_1_24_ = self.conv53_1_24_(relu52_1_24_)
        batchnorm53_1_24_ = self.batchnorm53_1_24_(conv53_1_24_)
        conv51_1_25_ = self.conv51_1_25_(relu49_)
        batchnorm51_1_25_ = self.batchnorm51_1_25_(conv51_1_25_)
        relu51_1_25_ = self.relu51_1_25_(batchnorm51_1_25_)
        conv52_1_25_padding = self.conv52_1_25_padding(relu51_1_25_)
        conv52_1_25_ = self.conv52_1_25_(conv52_1_25_padding)
        batchnorm52_1_25_ = self.batchnorm52_1_25_(conv52_1_25_)
        relu52_1_25_ = self.relu52_1_25_(batchnorm52_1_25_)
        conv53_1_25_ = self.conv53_1_25_(relu52_1_25_)
        batchnorm53_1_25_ = self.batchnorm53_1_25_(conv53_1_25_)
        conv51_1_26_ = self.conv51_1_26_(relu49_)
        batchnorm51_1_26_ = self.batchnorm51_1_26_(conv51_1_26_)
        relu51_1_26_ = self.relu51_1_26_(batchnorm51_1_26_)
        conv52_1_26_padding = self.conv52_1_26_padding(relu51_1_26_)
        conv52_1_26_ = self.conv52_1_26_(conv52_1_26_padding)
        batchnorm52_1_26_ = self.batchnorm52_1_26_(conv52_1_26_)
        relu52_1_26_ = self.relu52_1_26_(batchnorm52_1_26_)
        conv53_1_26_ = self.conv53_1_26_(relu52_1_26_)
        batchnorm53_1_26_ = self.batchnorm53_1_26_(conv53_1_26_)
        conv51_1_27_ = self.conv51_1_27_(relu49_)
        batchnorm51_1_27_ = self.batchnorm51_1_27_(conv51_1_27_)
        relu51_1_27_ = self.relu51_1_27_(batchnorm51_1_27_)
        conv52_1_27_padding = self.conv52_1_27_padding(relu51_1_27_)
        conv52_1_27_ = self.conv52_1_27_(conv52_1_27_padding)
        batchnorm52_1_27_ = self.batchnorm52_1_27_(conv52_1_27_)
        relu52_1_27_ = self.relu52_1_27_(batchnorm52_1_27_)
        conv53_1_27_ = self.conv53_1_27_(relu52_1_27_)
        batchnorm53_1_27_ = self.batchnorm53_1_27_(conv53_1_27_)
        conv51_1_28_ = self.conv51_1_28_(relu49_)
        batchnorm51_1_28_ = self.batchnorm51_1_28_(conv51_1_28_)
        relu51_1_28_ = self.relu51_1_28_(batchnorm51_1_28_)
        conv52_1_28_padding = self.conv52_1_28_padding(relu51_1_28_)
        conv52_1_28_ = self.conv52_1_28_(conv52_1_28_padding)
        batchnorm52_1_28_ = self.batchnorm52_1_28_(conv52_1_28_)
        relu52_1_28_ = self.relu52_1_28_(batchnorm52_1_28_)
        conv53_1_28_ = self.conv53_1_28_(relu52_1_28_)
        batchnorm53_1_28_ = self.batchnorm53_1_28_(conv53_1_28_)
        conv51_1_29_ = self.conv51_1_29_(relu49_)
        batchnorm51_1_29_ = self.batchnorm51_1_29_(conv51_1_29_)
        relu51_1_29_ = self.relu51_1_29_(batchnorm51_1_29_)
        conv52_1_29_padding = self.conv52_1_29_padding(relu51_1_29_)
        conv52_1_29_ = self.conv52_1_29_(conv52_1_29_padding)
        batchnorm52_1_29_ = self.batchnorm52_1_29_(conv52_1_29_)
        relu52_1_29_ = self.relu52_1_29_(batchnorm52_1_29_)
        conv53_1_29_ = self.conv53_1_29_(relu52_1_29_)
        batchnorm53_1_29_ = self.batchnorm53_1_29_(conv53_1_29_)
        conv51_1_30_ = self.conv51_1_30_(relu49_)
        batchnorm51_1_30_ = self.batchnorm51_1_30_(conv51_1_30_)
        relu51_1_30_ = self.relu51_1_30_(batchnorm51_1_30_)
        conv52_1_30_padding = self.conv52_1_30_padding(relu51_1_30_)
        conv52_1_30_ = self.conv52_1_30_(conv52_1_30_padding)
        batchnorm52_1_30_ = self.batchnorm52_1_30_(conv52_1_30_)
        relu52_1_30_ = self.relu52_1_30_(batchnorm52_1_30_)
        conv53_1_30_ = self.conv53_1_30_(relu52_1_30_)
        batchnorm53_1_30_ = self.batchnorm53_1_30_(conv53_1_30_)
        conv51_1_31_ = self.conv51_1_31_(relu49_)
        batchnorm51_1_31_ = self.batchnorm51_1_31_(conv51_1_31_)
        relu51_1_31_ = self.relu51_1_31_(batchnorm51_1_31_)
        conv52_1_31_padding = self.conv52_1_31_padding(relu51_1_31_)
        conv52_1_31_ = self.conv52_1_31_(conv52_1_31_padding)
        batchnorm52_1_31_ = self.batchnorm52_1_31_(conv52_1_31_)
        relu52_1_31_ = self.relu52_1_31_(batchnorm52_1_31_)
        conv53_1_31_ = self.conv53_1_31_(relu52_1_31_)
        batchnorm53_1_31_ = self.batchnorm53_1_31_(conv53_1_31_)
        conv51_1_32_ = self.conv51_1_32_(relu49_)
        batchnorm51_1_32_ = self.batchnorm51_1_32_(conv51_1_32_)
        relu51_1_32_ = self.relu51_1_32_(batchnorm51_1_32_)
        conv52_1_32_padding = self.conv52_1_32_padding(relu51_1_32_)
        conv52_1_32_ = self.conv52_1_32_(conv52_1_32_padding)
        batchnorm52_1_32_ = self.batchnorm52_1_32_(conv52_1_32_)
        relu52_1_32_ = self.relu52_1_32_(batchnorm52_1_32_)
        conv53_1_32_ = self.conv53_1_32_(relu52_1_32_)
        batchnorm53_1_32_ = self.batchnorm53_1_32_(conv53_1_32_)
        add54_1_ = batchnorm53_1_1_ + batchnorm53_1_2_ + batchnorm53_1_3_ + batchnorm53_1_4_ + batchnorm53_1_5_ + batchnorm53_1_6_ + batchnorm53_1_7_ + batchnorm53_1_8_ + batchnorm53_1_9_ + batchnorm53_1_10_ + batchnorm53_1_11_ + batchnorm53_1_12_ + batchnorm53_1_13_ + batchnorm53_1_14_ + batchnorm53_1_15_ + batchnorm53_1_16_ + batchnorm53_1_17_ + batchnorm53_1_18_ + batchnorm53_1_19_ + batchnorm53_1_20_ + batchnorm53_1_21_ + batchnorm53_1_22_ + batchnorm53_1_23_ + batchnorm53_1_24_ + batchnorm53_1_25_ + batchnorm53_1_26_ + batchnorm53_1_27_ + batchnorm53_1_28_ + batchnorm53_1_29_ + batchnorm53_1_30_ + batchnorm53_1_31_ + batchnorm53_1_32_
        add55_ = add54_1_ + relu49_
        relu55_ = self.relu55_(add55_)
        conv57_1_1_ = self.conv57_1_1_(relu55_)
        batchnorm57_1_1_ = self.batchnorm57_1_1_(conv57_1_1_)
        relu57_1_1_ = self.relu57_1_1_(batchnorm57_1_1_)
        conv58_1_1_padding = self.conv58_1_1_padding(relu57_1_1_)
        conv58_1_1_ = self.conv58_1_1_(conv58_1_1_padding)
        batchnorm58_1_1_ = self.batchnorm58_1_1_(conv58_1_1_)
        relu58_1_1_ = self.relu58_1_1_(batchnorm58_1_1_)
        conv59_1_1_ = self.conv59_1_1_(relu58_1_1_)
        batchnorm59_1_1_ = self.batchnorm59_1_1_(conv59_1_1_)
        conv57_1_2_ = self.conv57_1_2_(relu55_)
        batchnorm57_1_2_ = self.batchnorm57_1_2_(conv57_1_2_)
        relu57_1_2_ = self.relu57_1_2_(batchnorm57_1_2_)
        conv58_1_2_padding = self.conv58_1_2_padding(relu57_1_2_)
        conv58_1_2_ = self.conv58_1_2_(conv58_1_2_padding)
        batchnorm58_1_2_ = self.batchnorm58_1_2_(conv58_1_2_)
        relu58_1_2_ = self.relu58_1_2_(batchnorm58_1_2_)
        conv59_1_2_ = self.conv59_1_2_(relu58_1_2_)
        batchnorm59_1_2_ = self.batchnorm59_1_2_(conv59_1_2_)
        conv57_1_3_ = self.conv57_1_3_(relu55_)
        batchnorm57_1_3_ = self.batchnorm57_1_3_(conv57_1_3_)
        relu57_1_3_ = self.relu57_1_3_(batchnorm57_1_3_)
        conv58_1_3_padding = self.conv58_1_3_padding(relu57_1_3_)
        conv58_1_3_ = self.conv58_1_3_(conv58_1_3_padding)
        batchnorm58_1_3_ = self.batchnorm58_1_3_(conv58_1_3_)
        relu58_1_3_ = self.relu58_1_3_(batchnorm58_1_3_)
        conv59_1_3_ = self.conv59_1_3_(relu58_1_3_)
        batchnorm59_1_3_ = self.batchnorm59_1_3_(conv59_1_3_)
        conv57_1_4_ = self.conv57_1_4_(relu55_)
        batchnorm57_1_4_ = self.batchnorm57_1_4_(conv57_1_4_)
        relu57_1_4_ = self.relu57_1_4_(batchnorm57_1_4_)
        conv58_1_4_padding = self.conv58_1_4_padding(relu57_1_4_)
        conv58_1_4_ = self.conv58_1_4_(conv58_1_4_padding)
        batchnorm58_1_4_ = self.batchnorm58_1_4_(conv58_1_4_)
        relu58_1_4_ = self.relu58_1_4_(batchnorm58_1_4_)
        conv59_1_4_ = self.conv59_1_4_(relu58_1_4_)
        batchnorm59_1_4_ = self.batchnorm59_1_4_(conv59_1_4_)
        conv57_1_5_ = self.conv57_1_5_(relu55_)
        batchnorm57_1_5_ = self.batchnorm57_1_5_(conv57_1_5_)
        relu57_1_5_ = self.relu57_1_5_(batchnorm57_1_5_)
        conv58_1_5_padding = self.conv58_1_5_padding(relu57_1_5_)
        conv58_1_5_ = self.conv58_1_5_(conv58_1_5_padding)
        batchnorm58_1_5_ = self.batchnorm58_1_5_(conv58_1_5_)
        relu58_1_5_ = self.relu58_1_5_(batchnorm58_1_5_)
        conv59_1_5_ = self.conv59_1_5_(relu58_1_5_)
        batchnorm59_1_5_ = self.batchnorm59_1_5_(conv59_1_5_)
        conv57_1_6_ = self.conv57_1_6_(relu55_)
        batchnorm57_1_6_ = self.batchnorm57_1_6_(conv57_1_6_)
        relu57_1_6_ = self.relu57_1_6_(batchnorm57_1_6_)
        conv58_1_6_padding = self.conv58_1_6_padding(relu57_1_6_)
        conv58_1_6_ = self.conv58_1_6_(conv58_1_6_padding)
        batchnorm58_1_6_ = self.batchnorm58_1_6_(conv58_1_6_)
        relu58_1_6_ = self.relu58_1_6_(batchnorm58_1_6_)
        conv59_1_6_ = self.conv59_1_6_(relu58_1_6_)
        batchnorm59_1_6_ = self.batchnorm59_1_6_(conv59_1_6_)
        conv57_1_7_ = self.conv57_1_7_(relu55_)
        batchnorm57_1_7_ = self.batchnorm57_1_7_(conv57_1_7_)
        relu57_1_7_ = self.relu57_1_7_(batchnorm57_1_7_)
        conv58_1_7_padding = self.conv58_1_7_padding(relu57_1_7_)
        conv58_1_7_ = self.conv58_1_7_(conv58_1_7_padding)
        batchnorm58_1_7_ = self.batchnorm58_1_7_(conv58_1_7_)
        relu58_1_7_ = self.relu58_1_7_(batchnorm58_1_7_)
        conv59_1_7_ = self.conv59_1_7_(relu58_1_7_)
        batchnorm59_1_7_ = self.batchnorm59_1_7_(conv59_1_7_)
        conv57_1_8_ = self.conv57_1_8_(relu55_)
        batchnorm57_1_8_ = self.batchnorm57_1_8_(conv57_1_8_)
        relu57_1_8_ = self.relu57_1_8_(batchnorm57_1_8_)
        conv58_1_8_padding = self.conv58_1_8_padding(relu57_1_8_)
        conv58_1_8_ = self.conv58_1_8_(conv58_1_8_padding)
        batchnorm58_1_8_ = self.batchnorm58_1_8_(conv58_1_8_)
        relu58_1_8_ = self.relu58_1_8_(batchnorm58_1_8_)
        conv59_1_8_ = self.conv59_1_8_(relu58_1_8_)
        batchnorm59_1_8_ = self.batchnorm59_1_8_(conv59_1_8_)
        conv57_1_9_ = self.conv57_1_9_(relu55_)
        batchnorm57_1_9_ = self.batchnorm57_1_9_(conv57_1_9_)
        relu57_1_9_ = self.relu57_1_9_(batchnorm57_1_9_)
        conv58_1_9_padding = self.conv58_1_9_padding(relu57_1_9_)
        conv58_1_9_ = self.conv58_1_9_(conv58_1_9_padding)
        batchnorm58_1_9_ = self.batchnorm58_1_9_(conv58_1_9_)
        relu58_1_9_ = self.relu58_1_9_(batchnorm58_1_9_)
        conv59_1_9_ = self.conv59_1_9_(relu58_1_9_)
        batchnorm59_1_9_ = self.batchnorm59_1_9_(conv59_1_9_)
        conv57_1_10_ = self.conv57_1_10_(relu55_)
        batchnorm57_1_10_ = self.batchnorm57_1_10_(conv57_1_10_)
        relu57_1_10_ = self.relu57_1_10_(batchnorm57_1_10_)
        conv58_1_10_padding = self.conv58_1_10_padding(relu57_1_10_)
        conv58_1_10_ = self.conv58_1_10_(conv58_1_10_padding)
        batchnorm58_1_10_ = self.batchnorm58_1_10_(conv58_1_10_)
        relu58_1_10_ = self.relu58_1_10_(batchnorm58_1_10_)
        conv59_1_10_ = self.conv59_1_10_(relu58_1_10_)
        batchnorm59_1_10_ = self.batchnorm59_1_10_(conv59_1_10_)
        conv57_1_11_ = self.conv57_1_11_(relu55_)
        batchnorm57_1_11_ = self.batchnorm57_1_11_(conv57_1_11_)
        relu57_1_11_ = self.relu57_1_11_(batchnorm57_1_11_)
        conv58_1_11_padding = self.conv58_1_11_padding(relu57_1_11_)
        conv58_1_11_ = self.conv58_1_11_(conv58_1_11_padding)
        batchnorm58_1_11_ = self.batchnorm58_1_11_(conv58_1_11_)
        relu58_1_11_ = self.relu58_1_11_(batchnorm58_1_11_)
        conv59_1_11_ = self.conv59_1_11_(relu58_1_11_)
        batchnorm59_1_11_ = self.batchnorm59_1_11_(conv59_1_11_)
        conv57_1_12_ = self.conv57_1_12_(relu55_)
        batchnorm57_1_12_ = self.batchnorm57_1_12_(conv57_1_12_)
        relu57_1_12_ = self.relu57_1_12_(batchnorm57_1_12_)
        conv58_1_12_padding = self.conv58_1_12_padding(relu57_1_12_)
        conv58_1_12_ = self.conv58_1_12_(conv58_1_12_padding)
        batchnorm58_1_12_ = self.batchnorm58_1_12_(conv58_1_12_)
        relu58_1_12_ = self.relu58_1_12_(batchnorm58_1_12_)
        conv59_1_12_ = self.conv59_1_12_(relu58_1_12_)
        batchnorm59_1_12_ = self.batchnorm59_1_12_(conv59_1_12_)
        conv57_1_13_ = self.conv57_1_13_(relu55_)
        batchnorm57_1_13_ = self.batchnorm57_1_13_(conv57_1_13_)
        relu57_1_13_ = self.relu57_1_13_(batchnorm57_1_13_)
        conv58_1_13_padding = self.conv58_1_13_padding(relu57_1_13_)
        conv58_1_13_ = self.conv58_1_13_(conv58_1_13_padding)
        batchnorm58_1_13_ = self.batchnorm58_1_13_(conv58_1_13_)
        relu58_1_13_ = self.relu58_1_13_(batchnorm58_1_13_)
        conv59_1_13_ = self.conv59_1_13_(relu58_1_13_)
        batchnorm59_1_13_ = self.batchnorm59_1_13_(conv59_1_13_)
        conv57_1_14_ = self.conv57_1_14_(relu55_)
        batchnorm57_1_14_ = self.batchnorm57_1_14_(conv57_1_14_)
        relu57_1_14_ = self.relu57_1_14_(batchnorm57_1_14_)
        conv58_1_14_padding = self.conv58_1_14_padding(relu57_1_14_)
        conv58_1_14_ = self.conv58_1_14_(conv58_1_14_padding)
        batchnorm58_1_14_ = self.batchnorm58_1_14_(conv58_1_14_)
        relu58_1_14_ = self.relu58_1_14_(batchnorm58_1_14_)
        conv59_1_14_ = self.conv59_1_14_(relu58_1_14_)
        batchnorm59_1_14_ = self.batchnorm59_1_14_(conv59_1_14_)
        conv57_1_15_ = self.conv57_1_15_(relu55_)
        batchnorm57_1_15_ = self.batchnorm57_1_15_(conv57_1_15_)
        relu57_1_15_ = self.relu57_1_15_(batchnorm57_1_15_)
        conv58_1_15_padding = self.conv58_1_15_padding(relu57_1_15_)
        conv58_1_15_ = self.conv58_1_15_(conv58_1_15_padding)
        batchnorm58_1_15_ = self.batchnorm58_1_15_(conv58_1_15_)
        relu58_1_15_ = self.relu58_1_15_(batchnorm58_1_15_)
        conv59_1_15_ = self.conv59_1_15_(relu58_1_15_)
        batchnorm59_1_15_ = self.batchnorm59_1_15_(conv59_1_15_)
        conv57_1_16_ = self.conv57_1_16_(relu55_)
        batchnorm57_1_16_ = self.batchnorm57_1_16_(conv57_1_16_)
        relu57_1_16_ = self.relu57_1_16_(batchnorm57_1_16_)
        conv58_1_16_padding = self.conv58_1_16_padding(relu57_1_16_)
        conv58_1_16_ = self.conv58_1_16_(conv58_1_16_padding)
        batchnorm58_1_16_ = self.batchnorm58_1_16_(conv58_1_16_)
        relu58_1_16_ = self.relu58_1_16_(batchnorm58_1_16_)
        conv59_1_16_ = self.conv59_1_16_(relu58_1_16_)
        batchnorm59_1_16_ = self.batchnorm59_1_16_(conv59_1_16_)
        conv57_1_17_ = self.conv57_1_17_(relu55_)
        batchnorm57_1_17_ = self.batchnorm57_1_17_(conv57_1_17_)
        relu57_1_17_ = self.relu57_1_17_(batchnorm57_1_17_)
        conv58_1_17_padding = self.conv58_1_17_padding(relu57_1_17_)
        conv58_1_17_ = self.conv58_1_17_(conv58_1_17_padding)
        batchnorm58_1_17_ = self.batchnorm58_1_17_(conv58_1_17_)
        relu58_1_17_ = self.relu58_1_17_(batchnorm58_1_17_)
        conv59_1_17_ = self.conv59_1_17_(relu58_1_17_)
        batchnorm59_1_17_ = self.batchnorm59_1_17_(conv59_1_17_)
        conv57_1_18_ = self.conv57_1_18_(relu55_)
        batchnorm57_1_18_ = self.batchnorm57_1_18_(conv57_1_18_)
        relu57_1_18_ = self.relu57_1_18_(batchnorm57_1_18_)
        conv58_1_18_padding = self.conv58_1_18_padding(relu57_1_18_)
        conv58_1_18_ = self.conv58_1_18_(conv58_1_18_padding)
        batchnorm58_1_18_ = self.batchnorm58_1_18_(conv58_1_18_)
        relu58_1_18_ = self.relu58_1_18_(batchnorm58_1_18_)
        conv59_1_18_ = self.conv59_1_18_(relu58_1_18_)
        batchnorm59_1_18_ = self.batchnorm59_1_18_(conv59_1_18_)
        conv57_1_19_ = self.conv57_1_19_(relu55_)
        batchnorm57_1_19_ = self.batchnorm57_1_19_(conv57_1_19_)
        relu57_1_19_ = self.relu57_1_19_(batchnorm57_1_19_)
        conv58_1_19_padding = self.conv58_1_19_padding(relu57_1_19_)
        conv58_1_19_ = self.conv58_1_19_(conv58_1_19_padding)
        batchnorm58_1_19_ = self.batchnorm58_1_19_(conv58_1_19_)
        relu58_1_19_ = self.relu58_1_19_(batchnorm58_1_19_)
        conv59_1_19_ = self.conv59_1_19_(relu58_1_19_)
        batchnorm59_1_19_ = self.batchnorm59_1_19_(conv59_1_19_)
        conv57_1_20_ = self.conv57_1_20_(relu55_)
        batchnorm57_1_20_ = self.batchnorm57_1_20_(conv57_1_20_)
        relu57_1_20_ = self.relu57_1_20_(batchnorm57_1_20_)
        conv58_1_20_padding = self.conv58_1_20_padding(relu57_1_20_)
        conv58_1_20_ = self.conv58_1_20_(conv58_1_20_padding)
        batchnorm58_1_20_ = self.batchnorm58_1_20_(conv58_1_20_)
        relu58_1_20_ = self.relu58_1_20_(batchnorm58_1_20_)
        conv59_1_20_ = self.conv59_1_20_(relu58_1_20_)
        batchnorm59_1_20_ = self.batchnorm59_1_20_(conv59_1_20_)
        conv57_1_21_ = self.conv57_1_21_(relu55_)
        batchnorm57_1_21_ = self.batchnorm57_1_21_(conv57_1_21_)
        relu57_1_21_ = self.relu57_1_21_(batchnorm57_1_21_)
        conv58_1_21_padding = self.conv58_1_21_padding(relu57_1_21_)
        conv58_1_21_ = self.conv58_1_21_(conv58_1_21_padding)
        batchnorm58_1_21_ = self.batchnorm58_1_21_(conv58_1_21_)
        relu58_1_21_ = self.relu58_1_21_(batchnorm58_1_21_)
        conv59_1_21_ = self.conv59_1_21_(relu58_1_21_)
        batchnorm59_1_21_ = self.batchnorm59_1_21_(conv59_1_21_)
        conv57_1_22_ = self.conv57_1_22_(relu55_)
        batchnorm57_1_22_ = self.batchnorm57_1_22_(conv57_1_22_)
        relu57_1_22_ = self.relu57_1_22_(batchnorm57_1_22_)
        conv58_1_22_padding = self.conv58_1_22_padding(relu57_1_22_)
        conv58_1_22_ = self.conv58_1_22_(conv58_1_22_padding)
        batchnorm58_1_22_ = self.batchnorm58_1_22_(conv58_1_22_)
        relu58_1_22_ = self.relu58_1_22_(batchnorm58_1_22_)
        conv59_1_22_ = self.conv59_1_22_(relu58_1_22_)
        batchnorm59_1_22_ = self.batchnorm59_1_22_(conv59_1_22_)
        conv57_1_23_ = self.conv57_1_23_(relu55_)
        batchnorm57_1_23_ = self.batchnorm57_1_23_(conv57_1_23_)
        relu57_1_23_ = self.relu57_1_23_(batchnorm57_1_23_)
        conv58_1_23_padding = self.conv58_1_23_padding(relu57_1_23_)
        conv58_1_23_ = self.conv58_1_23_(conv58_1_23_padding)
        batchnorm58_1_23_ = self.batchnorm58_1_23_(conv58_1_23_)
        relu58_1_23_ = self.relu58_1_23_(batchnorm58_1_23_)
        conv59_1_23_ = self.conv59_1_23_(relu58_1_23_)
        batchnorm59_1_23_ = self.batchnorm59_1_23_(conv59_1_23_)
        conv57_1_24_ = self.conv57_1_24_(relu55_)
        batchnorm57_1_24_ = self.batchnorm57_1_24_(conv57_1_24_)
        relu57_1_24_ = self.relu57_1_24_(batchnorm57_1_24_)
        conv58_1_24_padding = self.conv58_1_24_padding(relu57_1_24_)
        conv58_1_24_ = self.conv58_1_24_(conv58_1_24_padding)
        batchnorm58_1_24_ = self.batchnorm58_1_24_(conv58_1_24_)
        relu58_1_24_ = self.relu58_1_24_(batchnorm58_1_24_)
        conv59_1_24_ = self.conv59_1_24_(relu58_1_24_)
        batchnorm59_1_24_ = self.batchnorm59_1_24_(conv59_1_24_)
        conv57_1_25_ = self.conv57_1_25_(relu55_)
        batchnorm57_1_25_ = self.batchnorm57_1_25_(conv57_1_25_)
        relu57_1_25_ = self.relu57_1_25_(batchnorm57_1_25_)
        conv58_1_25_padding = self.conv58_1_25_padding(relu57_1_25_)
        conv58_1_25_ = self.conv58_1_25_(conv58_1_25_padding)
        batchnorm58_1_25_ = self.batchnorm58_1_25_(conv58_1_25_)
        relu58_1_25_ = self.relu58_1_25_(batchnorm58_1_25_)
        conv59_1_25_ = self.conv59_1_25_(relu58_1_25_)
        batchnorm59_1_25_ = self.batchnorm59_1_25_(conv59_1_25_)
        conv57_1_26_ = self.conv57_1_26_(relu55_)
        batchnorm57_1_26_ = self.batchnorm57_1_26_(conv57_1_26_)
        relu57_1_26_ = self.relu57_1_26_(batchnorm57_1_26_)
        conv58_1_26_padding = self.conv58_1_26_padding(relu57_1_26_)
        conv58_1_26_ = self.conv58_1_26_(conv58_1_26_padding)
        batchnorm58_1_26_ = self.batchnorm58_1_26_(conv58_1_26_)
        relu58_1_26_ = self.relu58_1_26_(batchnorm58_1_26_)
        conv59_1_26_ = self.conv59_1_26_(relu58_1_26_)
        batchnorm59_1_26_ = self.batchnorm59_1_26_(conv59_1_26_)
        conv57_1_27_ = self.conv57_1_27_(relu55_)
        batchnorm57_1_27_ = self.batchnorm57_1_27_(conv57_1_27_)
        relu57_1_27_ = self.relu57_1_27_(batchnorm57_1_27_)
        conv58_1_27_padding = self.conv58_1_27_padding(relu57_1_27_)
        conv58_1_27_ = self.conv58_1_27_(conv58_1_27_padding)
        batchnorm58_1_27_ = self.batchnorm58_1_27_(conv58_1_27_)
        relu58_1_27_ = self.relu58_1_27_(batchnorm58_1_27_)
        conv59_1_27_ = self.conv59_1_27_(relu58_1_27_)
        batchnorm59_1_27_ = self.batchnorm59_1_27_(conv59_1_27_)
        conv57_1_28_ = self.conv57_1_28_(relu55_)
        batchnorm57_1_28_ = self.batchnorm57_1_28_(conv57_1_28_)
        relu57_1_28_ = self.relu57_1_28_(batchnorm57_1_28_)
        conv58_1_28_padding = self.conv58_1_28_padding(relu57_1_28_)
        conv58_1_28_ = self.conv58_1_28_(conv58_1_28_padding)
        batchnorm58_1_28_ = self.batchnorm58_1_28_(conv58_1_28_)
        relu58_1_28_ = self.relu58_1_28_(batchnorm58_1_28_)
        conv59_1_28_ = self.conv59_1_28_(relu58_1_28_)
        batchnorm59_1_28_ = self.batchnorm59_1_28_(conv59_1_28_)
        conv57_1_29_ = self.conv57_1_29_(relu55_)
        batchnorm57_1_29_ = self.batchnorm57_1_29_(conv57_1_29_)
        relu57_1_29_ = self.relu57_1_29_(batchnorm57_1_29_)
        conv58_1_29_padding = self.conv58_1_29_padding(relu57_1_29_)
        conv58_1_29_ = self.conv58_1_29_(conv58_1_29_padding)
        batchnorm58_1_29_ = self.batchnorm58_1_29_(conv58_1_29_)
        relu58_1_29_ = self.relu58_1_29_(batchnorm58_1_29_)
        conv59_1_29_ = self.conv59_1_29_(relu58_1_29_)
        batchnorm59_1_29_ = self.batchnorm59_1_29_(conv59_1_29_)
        conv57_1_30_ = self.conv57_1_30_(relu55_)
        batchnorm57_1_30_ = self.batchnorm57_1_30_(conv57_1_30_)
        relu57_1_30_ = self.relu57_1_30_(batchnorm57_1_30_)
        conv58_1_30_padding = self.conv58_1_30_padding(relu57_1_30_)
        conv58_1_30_ = self.conv58_1_30_(conv58_1_30_padding)
        batchnorm58_1_30_ = self.batchnorm58_1_30_(conv58_1_30_)
        relu58_1_30_ = self.relu58_1_30_(batchnorm58_1_30_)
        conv59_1_30_ = self.conv59_1_30_(relu58_1_30_)
        batchnorm59_1_30_ = self.batchnorm59_1_30_(conv59_1_30_)
        conv57_1_31_ = self.conv57_1_31_(relu55_)
        batchnorm57_1_31_ = self.batchnorm57_1_31_(conv57_1_31_)
        relu57_1_31_ = self.relu57_1_31_(batchnorm57_1_31_)
        conv58_1_31_padding = self.conv58_1_31_padding(relu57_1_31_)
        conv58_1_31_ = self.conv58_1_31_(conv58_1_31_padding)
        batchnorm58_1_31_ = self.batchnorm58_1_31_(conv58_1_31_)
        relu58_1_31_ = self.relu58_1_31_(batchnorm58_1_31_)
        conv59_1_31_ = self.conv59_1_31_(relu58_1_31_)
        batchnorm59_1_31_ = self.batchnorm59_1_31_(conv59_1_31_)
        conv57_1_32_ = self.conv57_1_32_(relu55_)
        batchnorm57_1_32_ = self.batchnorm57_1_32_(conv57_1_32_)
        relu57_1_32_ = self.relu57_1_32_(batchnorm57_1_32_)
        conv58_1_32_padding = self.conv58_1_32_padding(relu57_1_32_)
        conv58_1_32_ = self.conv58_1_32_(conv58_1_32_padding)
        batchnorm58_1_32_ = self.batchnorm58_1_32_(conv58_1_32_)
        relu58_1_32_ = self.relu58_1_32_(batchnorm58_1_32_)
        conv59_1_32_ = self.conv59_1_32_(relu58_1_32_)
        batchnorm59_1_32_ = self.batchnorm59_1_32_(conv59_1_32_)
        add60_1_ = batchnorm59_1_1_ + batchnorm59_1_2_ + batchnorm59_1_3_ + batchnorm59_1_4_ + batchnorm59_1_5_ + batchnorm59_1_6_ + batchnorm59_1_7_ + batchnorm59_1_8_ + batchnorm59_1_9_ + batchnorm59_1_10_ + batchnorm59_1_11_ + batchnorm59_1_12_ + batchnorm59_1_13_ + batchnorm59_1_14_ + batchnorm59_1_15_ + batchnorm59_1_16_ + batchnorm59_1_17_ + batchnorm59_1_18_ + batchnorm59_1_19_ + batchnorm59_1_20_ + batchnorm59_1_21_ + batchnorm59_1_22_ + batchnorm59_1_23_ + batchnorm59_1_24_ + batchnorm59_1_25_ + batchnorm59_1_26_ + batchnorm59_1_27_ + batchnorm59_1_28_ + batchnorm59_1_29_ + batchnorm59_1_30_ + batchnorm59_1_31_ + batchnorm59_1_32_
        add61_ = add60_1_ + relu55_
        relu61_ = self.relu61_(add61_)
        conv63_1_1_ = self.conv63_1_1_(relu61_)
        batchnorm63_1_1_ = self.batchnorm63_1_1_(conv63_1_1_)
        relu63_1_1_ = self.relu63_1_1_(batchnorm63_1_1_)
        conv64_1_1_padding = self.conv64_1_1_padding(relu63_1_1_)
        conv64_1_1_ = self.conv64_1_1_(conv64_1_1_padding)
        batchnorm64_1_1_ = self.batchnorm64_1_1_(conv64_1_1_)
        relu64_1_1_ = self.relu64_1_1_(batchnorm64_1_1_)
        conv65_1_1_ = self.conv65_1_1_(relu64_1_1_)
        batchnorm65_1_1_ = self.batchnorm65_1_1_(conv65_1_1_)
        conv63_1_2_ = self.conv63_1_2_(relu61_)
        batchnorm63_1_2_ = self.batchnorm63_1_2_(conv63_1_2_)
        relu63_1_2_ = self.relu63_1_2_(batchnorm63_1_2_)
        conv64_1_2_padding = self.conv64_1_2_padding(relu63_1_2_)
        conv64_1_2_ = self.conv64_1_2_(conv64_1_2_padding)
        batchnorm64_1_2_ = self.batchnorm64_1_2_(conv64_1_2_)
        relu64_1_2_ = self.relu64_1_2_(batchnorm64_1_2_)
        conv65_1_2_ = self.conv65_1_2_(relu64_1_2_)
        batchnorm65_1_2_ = self.batchnorm65_1_2_(conv65_1_2_)
        conv63_1_3_ = self.conv63_1_3_(relu61_)
        batchnorm63_1_3_ = self.batchnorm63_1_3_(conv63_1_3_)
        relu63_1_3_ = self.relu63_1_3_(batchnorm63_1_3_)
        conv64_1_3_padding = self.conv64_1_3_padding(relu63_1_3_)
        conv64_1_3_ = self.conv64_1_3_(conv64_1_3_padding)
        batchnorm64_1_3_ = self.batchnorm64_1_3_(conv64_1_3_)
        relu64_1_3_ = self.relu64_1_3_(batchnorm64_1_3_)
        conv65_1_3_ = self.conv65_1_3_(relu64_1_3_)
        batchnorm65_1_3_ = self.batchnorm65_1_3_(conv65_1_3_)
        conv63_1_4_ = self.conv63_1_4_(relu61_)
        batchnorm63_1_4_ = self.batchnorm63_1_4_(conv63_1_4_)
        relu63_1_4_ = self.relu63_1_4_(batchnorm63_1_4_)
        conv64_1_4_padding = self.conv64_1_4_padding(relu63_1_4_)
        conv64_1_4_ = self.conv64_1_4_(conv64_1_4_padding)
        batchnorm64_1_4_ = self.batchnorm64_1_4_(conv64_1_4_)
        relu64_1_4_ = self.relu64_1_4_(batchnorm64_1_4_)
        conv65_1_4_ = self.conv65_1_4_(relu64_1_4_)
        batchnorm65_1_4_ = self.batchnorm65_1_4_(conv65_1_4_)
        conv63_1_5_ = self.conv63_1_5_(relu61_)
        batchnorm63_1_5_ = self.batchnorm63_1_5_(conv63_1_5_)
        relu63_1_5_ = self.relu63_1_5_(batchnorm63_1_5_)
        conv64_1_5_padding = self.conv64_1_5_padding(relu63_1_5_)
        conv64_1_5_ = self.conv64_1_5_(conv64_1_5_padding)
        batchnorm64_1_5_ = self.batchnorm64_1_5_(conv64_1_5_)
        relu64_1_5_ = self.relu64_1_5_(batchnorm64_1_5_)
        conv65_1_5_ = self.conv65_1_5_(relu64_1_5_)
        batchnorm65_1_5_ = self.batchnorm65_1_5_(conv65_1_5_)
        conv63_1_6_ = self.conv63_1_6_(relu61_)
        batchnorm63_1_6_ = self.batchnorm63_1_6_(conv63_1_6_)
        relu63_1_6_ = self.relu63_1_6_(batchnorm63_1_6_)
        conv64_1_6_padding = self.conv64_1_6_padding(relu63_1_6_)
        conv64_1_6_ = self.conv64_1_6_(conv64_1_6_padding)
        batchnorm64_1_6_ = self.batchnorm64_1_6_(conv64_1_6_)
        relu64_1_6_ = self.relu64_1_6_(batchnorm64_1_6_)
        conv65_1_6_ = self.conv65_1_6_(relu64_1_6_)
        batchnorm65_1_6_ = self.batchnorm65_1_6_(conv65_1_6_)
        conv63_1_7_ = self.conv63_1_7_(relu61_)
        batchnorm63_1_7_ = self.batchnorm63_1_7_(conv63_1_7_)
        relu63_1_7_ = self.relu63_1_7_(batchnorm63_1_7_)
        conv64_1_7_padding = self.conv64_1_7_padding(relu63_1_7_)
        conv64_1_7_ = self.conv64_1_7_(conv64_1_7_padding)
        batchnorm64_1_7_ = self.batchnorm64_1_7_(conv64_1_7_)
        relu64_1_7_ = self.relu64_1_7_(batchnorm64_1_7_)
        conv65_1_7_ = self.conv65_1_7_(relu64_1_7_)
        batchnorm65_1_7_ = self.batchnorm65_1_7_(conv65_1_7_)
        conv63_1_8_ = self.conv63_1_8_(relu61_)
        batchnorm63_1_8_ = self.batchnorm63_1_8_(conv63_1_8_)
        relu63_1_8_ = self.relu63_1_8_(batchnorm63_1_8_)
        conv64_1_8_padding = self.conv64_1_8_padding(relu63_1_8_)
        conv64_1_8_ = self.conv64_1_8_(conv64_1_8_padding)
        batchnorm64_1_8_ = self.batchnorm64_1_8_(conv64_1_8_)
        relu64_1_8_ = self.relu64_1_8_(batchnorm64_1_8_)
        conv65_1_8_ = self.conv65_1_8_(relu64_1_8_)
        batchnorm65_1_8_ = self.batchnorm65_1_8_(conv65_1_8_)
        conv63_1_9_ = self.conv63_1_9_(relu61_)
        batchnorm63_1_9_ = self.batchnorm63_1_9_(conv63_1_9_)
        relu63_1_9_ = self.relu63_1_9_(batchnorm63_1_9_)
        conv64_1_9_padding = self.conv64_1_9_padding(relu63_1_9_)
        conv64_1_9_ = self.conv64_1_9_(conv64_1_9_padding)
        batchnorm64_1_9_ = self.batchnorm64_1_9_(conv64_1_9_)
        relu64_1_9_ = self.relu64_1_9_(batchnorm64_1_9_)
        conv65_1_9_ = self.conv65_1_9_(relu64_1_9_)
        batchnorm65_1_9_ = self.batchnorm65_1_9_(conv65_1_9_)
        conv63_1_10_ = self.conv63_1_10_(relu61_)
        batchnorm63_1_10_ = self.batchnorm63_1_10_(conv63_1_10_)
        relu63_1_10_ = self.relu63_1_10_(batchnorm63_1_10_)
        conv64_1_10_padding = self.conv64_1_10_padding(relu63_1_10_)
        conv64_1_10_ = self.conv64_1_10_(conv64_1_10_padding)
        batchnorm64_1_10_ = self.batchnorm64_1_10_(conv64_1_10_)
        relu64_1_10_ = self.relu64_1_10_(batchnorm64_1_10_)
        conv65_1_10_ = self.conv65_1_10_(relu64_1_10_)
        batchnorm65_1_10_ = self.batchnorm65_1_10_(conv65_1_10_)
        conv63_1_11_ = self.conv63_1_11_(relu61_)
        batchnorm63_1_11_ = self.batchnorm63_1_11_(conv63_1_11_)
        relu63_1_11_ = self.relu63_1_11_(batchnorm63_1_11_)
        conv64_1_11_padding = self.conv64_1_11_padding(relu63_1_11_)
        conv64_1_11_ = self.conv64_1_11_(conv64_1_11_padding)
        batchnorm64_1_11_ = self.batchnorm64_1_11_(conv64_1_11_)
        relu64_1_11_ = self.relu64_1_11_(batchnorm64_1_11_)
        conv65_1_11_ = self.conv65_1_11_(relu64_1_11_)
        batchnorm65_1_11_ = self.batchnorm65_1_11_(conv65_1_11_)
        conv63_1_12_ = self.conv63_1_12_(relu61_)
        batchnorm63_1_12_ = self.batchnorm63_1_12_(conv63_1_12_)
        relu63_1_12_ = self.relu63_1_12_(batchnorm63_1_12_)
        conv64_1_12_padding = self.conv64_1_12_padding(relu63_1_12_)
        conv64_1_12_ = self.conv64_1_12_(conv64_1_12_padding)
        batchnorm64_1_12_ = self.batchnorm64_1_12_(conv64_1_12_)
        relu64_1_12_ = self.relu64_1_12_(batchnorm64_1_12_)
        conv65_1_12_ = self.conv65_1_12_(relu64_1_12_)
        batchnorm65_1_12_ = self.batchnorm65_1_12_(conv65_1_12_)
        conv63_1_13_ = self.conv63_1_13_(relu61_)
        batchnorm63_1_13_ = self.batchnorm63_1_13_(conv63_1_13_)
        relu63_1_13_ = self.relu63_1_13_(batchnorm63_1_13_)
        conv64_1_13_padding = self.conv64_1_13_padding(relu63_1_13_)
        conv64_1_13_ = self.conv64_1_13_(conv64_1_13_padding)
        batchnorm64_1_13_ = self.batchnorm64_1_13_(conv64_1_13_)
        relu64_1_13_ = self.relu64_1_13_(batchnorm64_1_13_)
        conv65_1_13_ = self.conv65_1_13_(relu64_1_13_)
        batchnorm65_1_13_ = self.batchnorm65_1_13_(conv65_1_13_)
        conv63_1_14_ = self.conv63_1_14_(relu61_)
        batchnorm63_1_14_ = self.batchnorm63_1_14_(conv63_1_14_)
        relu63_1_14_ = self.relu63_1_14_(batchnorm63_1_14_)
        conv64_1_14_padding = self.conv64_1_14_padding(relu63_1_14_)
        conv64_1_14_ = self.conv64_1_14_(conv64_1_14_padding)
        batchnorm64_1_14_ = self.batchnorm64_1_14_(conv64_1_14_)
        relu64_1_14_ = self.relu64_1_14_(batchnorm64_1_14_)
        conv65_1_14_ = self.conv65_1_14_(relu64_1_14_)
        batchnorm65_1_14_ = self.batchnorm65_1_14_(conv65_1_14_)
        conv63_1_15_ = self.conv63_1_15_(relu61_)
        batchnorm63_1_15_ = self.batchnorm63_1_15_(conv63_1_15_)
        relu63_1_15_ = self.relu63_1_15_(batchnorm63_1_15_)
        conv64_1_15_padding = self.conv64_1_15_padding(relu63_1_15_)
        conv64_1_15_ = self.conv64_1_15_(conv64_1_15_padding)
        batchnorm64_1_15_ = self.batchnorm64_1_15_(conv64_1_15_)
        relu64_1_15_ = self.relu64_1_15_(batchnorm64_1_15_)
        conv65_1_15_ = self.conv65_1_15_(relu64_1_15_)
        batchnorm65_1_15_ = self.batchnorm65_1_15_(conv65_1_15_)
        conv63_1_16_ = self.conv63_1_16_(relu61_)
        batchnorm63_1_16_ = self.batchnorm63_1_16_(conv63_1_16_)
        relu63_1_16_ = self.relu63_1_16_(batchnorm63_1_16_)
        conv64_1_16_padding = self.conv64_1_16_padding(relu63_1_16_)
        conv64_1_16_ = self.conv64_1_16_(conv64_1_16_padding)
        batchnorm64_1_16_ = self.batchnorm64_1_16_(conv64_1_16_)
        relu64_1_16_ = self.relu64_1_16_(batchnorm64_1_16_)
        conv65_1_16_ = self.conv65_1_16_(relu64_1_16_)
        batchnorm65_1_16_ = self.batchnorm65_1_16_(conv65_1_16_)
        conv63_1_17_ = self.conv63_1_17_(relu61_)
        batchnorm63_1_17_ = self.batchnorm63_1_17_(conv63_1_17_)
        relu63_1_17_ = self.relu63_1_17_(batchnorm63_1_17_)
        conv64_1_17_padding = self.conv64_1_17_padding(relu63_1_17_)
        conv64_1_17_ = self.conv64_1_17_(conv64_1_17_padding)
        batchnorm64_1_17_ = self.batchnorm64_1_17_(conv64_1_17_)
        relu64_1_17_ = self.relu64_1_17_(batchnorm64_1_17_)
        conv65_1_17_ = self.conv65_1_17_(relu64_1_17_)
        batchnorm65_1_17_ = self.batchnorm65_1_17_(conv65_1_17_)
        conv63_1_18_ = self.conv63_1_18_(relu61_)
        batchnorm63_1_18_ = self.batchnorm63_1_18_(conv63_1_18_)
        relu63_1_18_ = self.relu63_1_18_(batchnorm63_1_18_)
        conv64_1_18_padding = self.conv64_1_18_padding(relu63_1_18_)
        conv64_1_18_ = self.conv64_1_18_(conv64_1_18_padding)
        batchnorm64_1_18_ = self.batchnorm64_1_18_(conv64_1_18_)
        relu64_1_18_ = self.relu64_1_18_(batchnorm64_1_18_)
        conv65_1_18_ = self.conv65_1_18_(relu64_1_18_)
        batchnorm65_1_18_ = self.batchnorm65_1_18_(conv65_1_18_)
        conv63_1_19_ = self.conv63_1_19_(relu61_)
        batchnorm63_1_19_ = self.batchnorm63_1_19_(conv63_1_19_)
        relu63_1_19_ = self.relu63_1_19_(batchnorm63_1_19_)
        conv64_1_19_padding = self.conv64_1_19_padding(relu63_1_19_)
        conv64_1_19_ = self.conv64_1_19_(conv64_1_19_padding)
        batchnorm64_1_19_ = self.batchnorm64_1_19_(conv64_1_19_)
        relu64_1_19_ = self.relu64_1_19_(batchnorm64_1_19_)
        conv65_1_19_ = self.conv65_1_19_(relu64_1_19_)
        batchnorm65_1_19_ = self.batchnorm65_1_19_(conv65_1_19_)
        conv63_1_20_ = self.conv63_1_20_(relu61_)
        batchnorm63_1_20_ = self.batchnorm63_1_20_(conv63_1_20_)
        relu63_1_20_ = self.relu63_1_20_(batchnorm63_1_20_)
        conv64_1_20_padding = self.conv64_1_20_padding(relu63_1_20_)
        conv64_1_20_ = self.conv64_1_20_(conv64_1_20_padding)
        batchnorm64_1_20_ = self.batchnorm64_1_20_(conv64_1_20_)
        relu64_1_20_ = self.relu64_1_20_(batchnorm64_1_20_)
        conv65_1_20_ = self.conv65_1_20_(relu64_1_20_)
        batchnorm65_1_20_ = self.batchnorm65_1_20_(conv65_1_20_)
        conv63_1_21_ = self.conv63_1_21_(relu61_)
        batchnorm63_1_21_ = self.batchnorm63_1_21_(conv63_1_21_)
        relu63_1_21_ = self.relu63_1_21_(batchnorm63_1_21_)
        conv64_1_21_padding = self.conv64_1_21_padding(relu63_1_21_)
        conv64_1_21_ = self.conv64_1_21_(conv64_1_21_padding)
        batchnorm64_1_21_ = self.batchnorm64_1_21_(conv64_1_21_)
        relu64_1_21_ = self.relu64_1_21_(batchnorm64_1_21_)
        conv65_1_21_ = self.conv65_1_21_(relu64_1_21_)
        batchnorm65_1_21_ = self.batchnorm65_1_21_(conv65_1_21_)
        conv63_1_22_ = self.conv63_1_22_(relu61_)
        batchnorm63_1_22_ = self.batchnorm63_1_22_(conv63_1_22_)
        relu63_1_22_ = self.relu63_1_22_(batchnorm63_1_22_)
        conv64_1_22_padding = self.conv64_1_22_padding(relu63_1_22_)
        conv64_1_22_ = self.conv64_1_22_(conv64_1_22_padding)
        batchnorm64_1_22_ = self.batchnorm64_1_22_(conv64_1_22_)
        relu64_1_22_ = self.relu64_1_22_(batchnorm64_1_22_)
        conv65_1_22_ = self.conv65_1_22_(relu64_1_22_)
        batchnorm65_1_22_ = self.batchnorm65_1_22_(conv65_1_22_)
        conv63_1_23_ = self.conv63_1_23_(relu61_)
        batchnorm63_1_23_ = self.batchnorm63_1_23_(conv63_1_23_)
        relu63_1_23_ = self.relu63_1_23_(batchnorm63_1_23_)
        conv64_1_23_padding = self.conv64_1_23_padding(relu63_1_23_)
        conv64_1_23_ = self.conv64_1_23_(conv64_1_23_padding)
        batchnorm64_1_23_ = self.batchnorm64_1_23_(conv64_1_23_)
        relu64_1_23_ = self.relu64_1_23_(batchnorm64_1_23_)
        conv65_1_23_ = self.conv65_1_23_(relu64_1_23_)
        batchnorm65_1_23_ = self.batchnorm65_1_23_(conv65_1_23_)
        conv63_1_24_ = self.conv63_1_24_(relu61_)
        batchnorm63_1_24_ = self.batchnorm63_1_24_(conv63_1_24_)
        relu63_1_24_ = self.relu63_1_24_(batchnorm63_1_24_)
        conv64_1_24_padding = self.conv64_1_24_padding(relu63_1_24_)
        conv64_1_24_ = self.conv64_1_24_(conv64_1_24_padding)
        batchnorm64_1_24_ = self.batchnorm64_1_24_(conv64_1_24_)
        relu64_1_24_ = self.relu64_1_24_(batchnorm64_1_24_)
        conv65_1_24_ = self.conv65_1_24_(relu64_1_24_)
        batchnorm65_1_24_ = self.batchnorm65_1_24_(conv65_1_24_)
        conv63_1_25_ = self.conv63_1_25_(relu61_)
        batchnorm63_1_25_ = self.batchnorm63_1_25_(conv63_1_25_)
        relu63_1_25_ = self.relu63_1_25_(batchnorm63_1_25_)
        conv64_1_25_padding = self.conv64_1_25_padding(relu63_1_25_)
        conv64_1_25_ = self.conv64_1_25_(conv64_1_25_padding)
        batchnorm64_1_25_ = self.batchnorm64_1_25_(conv64_1_25_)
        relu64_1_25_ = self.relu64_1_25_(batchnorm64_1_25_)
        conv65_1_25_ = self.conv65_1_25_(relu64_1_25_)
        batchnorm65_1_25_ = self.batchnorm65_1_25_(conv65_1_25_)
        conv63_1_26_ = self.conv63_1_26_(relu61_)
        batchnorm63_1_26_ = self.batchnorm63_1_26_(conv63_1_26_)
        relu63_1_26_ = self.relu63_1_26_(batchnorm63_1_26_)
        conv64_1_26_padding = self.conv64_1_26_padding(relu63_1_26_)
        conv64_1_26_ = self.conv64_1_26_(conv64_1_26_padding)
        batchnorm64_1_26_ = self.batchnorm64_1_26_(conv64_1_26_)
        relu64_1_26_ = self.relu64_1_26_(batchnorm64_1_26_)
        conv65_1_26_ = self.conv65_1_26_(relu64_1_26_)
        batchnorm65_1_26_ = self.batchnorm65_1_26_(conv65_1_26_)
        conv63_1_27_ = self.conv63_1_27_(relu61_)
        batchnorm63_1_27_ = self.batchnorm63_1_27_(conv63_1_27_)
        relu63_1_27_ = self.relu63_1_27_(batchnorm63_1_27_)
        conv64_1_27_padding = self.conv64_1_27_padding(relu63_1_27_)
        conv64_1_27_ = self.conv64_1_27_(conv64_1_27_padding)
        batchnorm64_1_27_ = self.batchnorm64_1_27_(conv64_1_27_)
        relu64_1_27_ = self.relu64_1_27_(batchnorm64_1_27_)
        conv65_1_27_ = self.conv65_1_27_(relu64_1_27_)
        batchnorm65_1_27_ = self.batchnorm65_1_27_(conv65_1_27_)
        conv63_1_28_ = self.conv63_1_28_(relu61_)
        batchnorm63_1_28_ = self.batchnorm63_1_28_(conv63_1_28_)
        relu63_1_28_ = self.relu63_1_28_(batchnorm63_1_28_)
        conv64_1_28_padding = self.conv64_1_28_padding(relu63_1_28_)
        conv64_1_28_ = self.conv64_1_28_(conv64_1_28_padding)
        batchnorm64_1_28_ = self.batchnorm64_1_28_(conv64_1_28_)
        relu64_1_28_ = self.relu64_1_28_(batchnorm64_1_28_)
        conv65_1_28_ = self.conv65_1_28_(relu64_1_28_)
        batchnorm65_1_28_ = self.batchnorm65_1_28_(conv65_1_28_)
        conv63_1_29_ = self.conv63_1_29_(relu61_)
        batchnorm63_1_29_ = self.batchnorm63_1_29_(conv63_1_29_)
        relu63_1_29_ = self.relu63_1_29_(batchnorm63_1_29_)
        conv64_1_29_padding = self.conv64_1_29_padding(relu63_1_29_)
        conv64_1_29_ = self.conv64_1_29_(conv64_1_29_padding)
        batchnorm64_1_29_ = self.batchnorm64_1_29_(conv64_1_29_)
        relu64_1_29_ = self.relu64_1_29_(batchnorm64_1_29_)
        conv65_1_29_ = self.conv65_1_29_(relu64_1_29_)
        batchnorm65_1_29_ = self.batchnorm65_1_29_(conv65_1_29_)
        conv63_1_30_ = self.conv63_1_30_(relu61_)
        batchnorm63_1_30_ = self.batchnorm63_1_30_(conv63_1_30_)
        relu63_1_30_ = self.relu63_1_30_(batchnorm63_1_30_)
        conv64_1_30_padding = self.conv64_1_30_padding(relu63_1_30_)
        conv64_1_30_ = self.conv64_1_30_(conv64_1_30_padding)
        batchnorm64_1_30_ = self.batchnorm64_1_30_(conv64_1_30_)
        relu64_1_30_ = self.relu64_1_30_(batchnorm64_1_30_)
        conv65_1_30_ = self.conv65_1_30_(relu64_1_30_)
        batchnorm65_1_30_ = self.batchnorm65_1_30_(conv65_1_30_)
        conv63_1_31_ = self.conv63_1_31_(relu61_)
        batchnorm63_1_31_ = self.batchnorm63_1_31_(conv63_1_31_)
        relu63_1_31_ = self.relu63_1_31_(batchnorm63_1_31_)
        conv64_1_31_padding = self.conv64_1_31_padding(relu63_1_31_)
        conv64_1_31_ = self.conv64_1_31_(conv64_1_31_padding)
        batchnorm64_1_31_ = self.batchnorm64_1_31_(conv64_1_31_)
        relu64_1_31_ = self.relu64_1_31_(batchnorm64_1_31_)
        conv65_1_31_ = self.conv65_1_31_(relu64_1_31_)
        batchnorm65_1_31_ = self.batchnorm65_1_31_(conv65_1_31_)
        conv63_1_32_ = self.conv63_1_32_(relu61_)
        batchnorm63_1_32_ = self.batchnorm63_1_32_(conv63_1_32_)
        relu63_1_32_ = self.relu63_1_32_(batchnorm63_1_32_)
        conv64_1_32_padding = self.conv64_1_32_padding(relu63_1_32_)
        conv64_1_32_ = self.conv64_1_32_(conv64_1_32_padding)
        batchnorm64_1_32_ = self.batchnorm64_1_32_(conv64_1_32_)
        relu64_1_32_ = self.relu64_1_32_(batchnorm64_1_32_)
        conv65_1_32_ = self.conv65_1_32_(relu64_1_32_)
        batchnorm65_1_32_ = self.batchnorm65_1_32_(conv65_1_32_)
        add66_1_ = batchnorm65_1_1_ + batchnorm65_1_2_ + batchnorm65_1_3_ + batchnorm65_1_4_ + batchnorm65_1_5_ + batchnorm65_1_6_ + batchnorm65_1_7_ + batchnorm65_1_8_ + batchnorm65_1_9_ + batchnorm65_1_10_ + batchnorm65_1_11_ + batchnorm65_1_12_ + batchnorm65_1_13_ + batchnorm65_1_14_ + batchnorm65_1_15_ + batchnorm65_1_16_ + batchnorm65_1_17_ + batchnorm65_1_18_ + batchnorm65_1_19_ + batchnorm65_1_20_ + batchnorm65_1_21_ + batchnorm65_1_22_ + batchnorm65_1_23_ + batchnorm65_1_24_ + batchnorm65_1_25_ + batchnorm65_1_26_ + batchnorm65_1_27_ + batchnorm65_1_28_ + batchnorm65_1_29_ + batchnorm65_1_30_ + batchnorm65_1_31_ + batchnorm65_1_32_
        add67_ = add66_1_ + relu61_
        relu67_ = self.relu67_(add67_)
        conv69_1_1_ = self.conv69_1_1_(relu67_)
        batchnorm69_1_1_ = self.batchnorm69_1_1_(conv69_1_1_)
        relu69_1_1_ = self.relu69_1_1_(batchnorm69_1_1_)
        conv70_1_1_padding = self.conv70_1_1_padding(relu69_1_1_)
        conv70_1_1_ = self.conv70_1_1_(conv70_1_1_padding)
        batchnorm70_1_1_ = self.batchnorm70_1_1_(conv70_1_1_)
        relu70_1_1_ = self.relu70_1_1_(batchnorm70_1_1_)
        conv71_1_1_ = self.conv71_1_1_(relu70_1_1_)
        batchnorm71_1_1_ = self.batchnorm71_1_1_(conv71_1_1_)
        conv69_1_2_ = self.conv69_1_2_(relu67_)
        batchnorm69_1_2_ = self.batchnorm69_1_2_(conv69_1_2_)
        relu69_1_2_ = self.relu69_1_2_(batchnorm69_1_2_)
        conv70_1_2_padding = self.conv70_1_2_padding(relu69_1_2_)
        conv70_1_2_ = self.conv70_1_2_(conv70_1_2_padding)
        batchnorm70_1_2_ = self.batchnorm70_1_2_(conv70_1_2_)
        relu70_1_2_ = self.relu70_1_2_(batchnorm70_1_2_)
        conv71_1_2_ = self.conv71_1_2_(relu70_1_2_)
        batchnorm71_1_2_ = self.batchnorm71_1_2_(conv71_1_2_)
        conv69_1_3_ = self.conv69_1_3_(relu67_)
        batchnorm69_1_3_ = self.batchnorm69_1_3_(conv69_1_3_)
        relu69_1_3_ = self.relu69_1_3_(batchnorm69_1_3_)
        conv70_1_3_padding = self.conv70_1_3_padding(relu69_1_3_)
        conv70_1_3_ = self.conv70_1_3_(conv70_1_3_padding)
        batchnorm70_1_3_ = self.batchnorm70_1_3_(conv70_1_3_)
        relu70_1_3_ = self.relu70_1_3_(batchnorm70_1_3_)
        conv71_1_3_ = self.conv71_1_3_(relu70_1_3_)
        batchnorm71_1_3_ = self.batchnorm71_1_3_(conv71_1_3_)
        conv69_1_4_ = self.conv69_1_4_(relu67_)
        batchnorm69_1_4_ = self.batchnorm69_1_4_(conv69_1_4_)
        relu69_1_4_ = self.relu69_1_4_(batchnorm69_1_4_)
        conv70_1_4_padding = self.conv70_1_4_padding(relu69_1_4_)
        conv70_1_4_ = self.conv70_1_4_(conv70_1_4_padding)
        batchnorm70_1_4_ = self.batchnorm70_1_4_(conv70_1_4_)
        relu70_1_4_ = self.relu70_1_4_(batchnorm70_1_4_)
        conv71_1_4_ = self.conv71_1_4_(relu70_1_4_)
        batchnorm71_1_4_ = self.batchnorm71_1_4_(conv71_1_4_)
        conv69_1_5_ = self.conv69_1_5_(relu67_)
        batchnorm69_1_5_ = self.batchnorm69_1_5_(conv69_1_5_)
        relu69_1_5_ = self.relu69_1_5_(batchnorm69_1_5_)
        conv70_1_5_padding = self.conv70_1_5_padding(relu69_1_5_)
        conv70_1_5_ = self.conv70_1_5_(conv70_1_5_padding)
        batchnorm70_1_5_ = self.batchnorm70_1_5_(conv70_1_5_)
        relu70_1_5_ = self.relu70_1_5_(batchnorm70_1_5_)
        conv71_1_5_ = self.conv71_1_5_(relu70_1_5_)
        batchnorm71_1_5_ = self.batchnorm71_1_5_(conv71_1_5_)
        conv69_1_6_ = self.conv69_1_6_(relu67_)
        batchnorm69_1_6_ = self.batchnorm69_1_6_(conv69_1_6_)
        relu69_1_6_ = self.relu69_1_6_(batchnorm69_1_6_)
        conv70_1_6_padding = self.conv70_1_6_padding(relu69_1_6_)
        conv70_1_6_ = self.conv70_1_6_(conv70_1_6_padding)
        batchnorm70_1_6_ = self.batchnorm70_1_6_(conv70_1_6_)
        relu70_1_6_ = self.relu70_1_6_(batchnorm70_1_6_)
        conv71_1_6_ = self.conv71_1_6_(relu70_1_6_)
        batchnorm71_1_6_ = self.batchnorm71_1_6_(conv71_1_6_)
        conv69_1_7_ = self.conv69_1_7_(relu67_)
        batchnorm69_1_7_ = self.batchnorm69_1_7_(conv69_1_7_)
        relu69_1_7_ = self.relu69_1_7_(batchnorm69_1_7_)
        conv70_1_7_padding = self.conv70_1_7_padding(relu69_1_7_)
        conv70_1_7_ = self.conv70_1_7_(conv70_1_7_padding)
        batchnorm70_1_7_ = self.batchnorm70_1_7_(conv70_1_7_)
        relu70_1_7_ = self.relu70_1_7_(batchnorm70_1_7_)
        conv71_1_7_ = self.conv71_1_7_(relu70_1_7_)
        batchnorm71_1_7_ = self.batchnorm71_1_7_(conv71_1_7_)
        conv69_1_8_ = self.conv69_1_8_(relu67_)
        batchnorm69_1_8_ = self.batchnorm69_1_8_(conv69_1_8_)
        relu69_1_8_ = self.relu69_1_8_(batchnorm69_1_8_)
        conv70_1_8_padding = self.conv70_1_8_padding(relu69_1_8_)
        conv70_1_8_ = self.conv70_1_8_(conv70_1_8_padding)
        batchnorm70_1_8_ = self.batchnorm70_1_8_(conv70_1_8_)
        relu70_1_8_ = self.relu70_1_8_(batchnorm70_1_8_)
        conv71_1_8_ = self.conv71_1_8_(relu70_1_8_)
        batchnorm71_1_8_ = self.batchnorm71_1_8_(conv71_1_8_)
        conv69_1_9_ = self.conv69_1_9_(relu67_)
        batchnorm69_1_9_ = self.batchnorm69_1_9_(conv69_1_9_)
        relu69_1_9_ = self.relu69_1_9_(batchnorm69_1_9_)
        conv70_1_9_padding = self.conv70_1_9_padding(relu69_1_9_)
        conv70_1_9_ = self.conv70_1_9_(conv70_1_9_padding)
        batchnorm70_1_9_ = self.batchnorm70_1_9_(conv70_1_9_)
        relu70_1_9_ = self.relu70_1_9_(batchnorm70_1_9_)
        conv71_1_9_ = self.conv71_1_9_(relu70_1_9_)
        batchnorm71_1_9_ = self.batchnorm71_1_9_(conv71_1_9_)
        conv69_1_10_ = self.conv69_1_10_(relu67_)
        batchnorm69_1_10_ = self.batchnorm69_1_10_(conv69_1_10_)
        relu69_1_10_ = self.relu69_1_10_(batchnorm69_1_10_)
        conv70_1_10_padding = self.conv70_1_10_padding(relu69_1_10_)
        conv70_1_10_ = self.conv70_1_10_(conv70_1_10_padding)
        batchnorm70_1_10_ = self.batchnorm70_1_10_(conv70_1_10_)
        relu70_1_10_ = self.relu70_1_10_(batchnorm70_1_10_)
        conv71_1_10_ = self.conv71_1_10_(relu70_1_10_)
        batchnorm71_1_10_ = self.batchnorm71_1_10_(conv71_1_10_)
        conv69_1_11_ = self.conv69_1_11_(relu67_)
        batchnorm69_1_11_ = self.batchnorm69_1_11_(conv69_1_11_)
        relu69_1_11_ = self.relu69_1_11_(batchnorm69_1_11_)
        conv70_1_11_padding = self.conv70_1_11_padding(relu69_1_11_)
        conv70_1_11_ = self.conv70_1_11_(conv70_1_11_padding)
        batchnorm70_1_11_ = self.batchnorm70_1_11_(conv70_1_11_)
        relu70_1_11_ = self.relu70_1_11_(batchnorm70_1_11_)
        conv71_1_11_ = self.conv71_1_11_(relu70_1_11_)
        batchnorm71_1_11_ = self.batchnorm71_1_11_(conv71_1_11_)
        conv69_1_12_ = self.conv69_1_12_(relu67_)
        batchnorm69_1_12_ = self.batchnorm69_1_12_(conv69_1_12_)
        relu69_1_12_ = self.relu69_1_12_(batchnorm69_1_12_)
        conv70_1_12_padding = self.conv70_1_12_padding(relu69_1_12_)
        conv70_1_12_ = self.conv70_1_12_(conv70_1_12_padding)
        batchnorm70_1_12_ = self.batchnorm70_1_12_(conv70_1_12_)
        relu70_1_12_ = self.relu70_1_12_(batchnorm70_1_12_)
        conv71_1_12_ = self.conv71_1_12_(relu70_1_12_)
        batchnorm71_1_12_ = self.batchnorm71_1_12_(conv71_1_12_)
        conv69_1_13_ = self.conv69_1_13_(relu67_)
        batchnorm69_1_13_ = self.batchnorm69_1_13_(conv69_1_13_)
        relu69_1_13_ = self.relu69_1_13_(batchnorm69_1_13_)
        conv70_1_13_padding = self.conv70_1_13_padding(relu69_1_13_)
        conv70_1_13_ = self.conv70_1_13_(conv70_1_13_padding)
        batchnorm70_1_13_ = self.batchnorm70_1_13_(conv70_1_13_)
        relu70_1_13_ = self.relu70_1_13_(batchnorm70_1_13_)
        conv71_1_13_ = self.conv71_1_13_(relu70_1_13_)
        batchnorm71_1_13_ = self.batchnorm71_1_13_(conv71_1_13_)
        conv69_1_14_ = self.conv69_1_14_(relu67_)
        batchnorm69_1_14_ = self.batchnorm69_1_14_(conv69_1_14_)
        relu69_1_14_ = self.relu69_1_14_(batchnorm69_1_14_)
        conv70_1_14_padding = self.conv70_1_14_padding(relu69_1_14_)
        conv70_1_14_ = self.conv70_1_14_(conv70_1_14_padding)
        batchnorm70_1_14_ = self.batchnorm70_1_14_(conv70_1_14_)
        relu70_1_14_ = self.relu70_1_14_(batchnorm70_1_14_)
        conv71_1_14_ = self.conv71_1_14_(relu70_1_14_)
        batchnorm71_1_14_ = self.batchnorm71_1_14_(conv71_1_14_)
        conv69_1_15_ = self.conv69_1_15_(relu67_)
        batchnorm69_1_15_ = self.batchnorm69_1_15_(conv69_1_15_)
        relu69_1_15_ = self.relu69_1_15_(batchnorm69_1_15_)
        conv70_1_15_padding = self.conv70_1_15_padding(relu69_1_15_)
        conv70_1_15_ = self.conv70_1_15_(conv70_1_15_padding)
        batchnorm70_1_15_ = self.batchnorm70_1_15_(conv70_1_15_)
        relu70_1_15_ = self.relu70_1_15_(batchnorm70_1_15_)
        conv71_1_15_ = self.conv71_1_15_(relu70_1_15_)
        batchnorm71_1_15_ = self.batchnorm71_1_15_(conv71_1_15_)
        conv69_1_16_ = self.conv69_1_16_(relu67_)
        batchnorm69_1_16_ = self.batchnorm69_1_16_(conv69_1_16_)
        relu69_1_16_ = self.relu69_1_16_(batchnorm69_1_16_)
        conv70_1_16_padding = self.conv70_1_16_padding(relu69_1_16_)
        conv70_1_16_ = self.conv70_1_16_(conv70_1_16_padding)
        batchnorm70_1_16_ = self.batchnorm70_1_16_(conv70_1_16_)
        relu70_1_16_ = self.relu70_1_16_(batchnorm70_1_16_)
        conv71_1_16_ = self.conv71_1_16_(relu70_1_16_)
        batchnorm71_1_16_ = self.batchnorm71_1_16_(conv71_1_16_)
        conv69_1_17_ = self.conv69_1_17_(relu67_)
        batchnorm69_1_17_ = self.batchnorm69_1_17_(conv69_1_17_)
        relu69_1_17_ = self.relu69_1_17_(batchnorm69_1_17_)
        conv70_1_17_padding = self.conv70_1_17_padding(relu69_1_17_)
        conv70_1_17_ = self.conv70_1_17_(conv70_1_17_padding)
        batchnorm70_1_17_ = self.batchnorm70_1_17_(conv70_1_17_)
        relu70_1_17_ = self.relu70_1_17_(batchnorm70_1_17_)
        conv71_1_17_ = self.conv71_1_17_(relu70_1_17_)
        batchnorm71_1_17_ = self.batchnorm71_1_17_(conv71_1_17_)
        conv69_1_18_ = self.conv69_1_18_(relu67_)
        batchnorm69_1_18_ = self.batchnorm69_1_18_(conv69_1_18_)
        relu69_1_18_ = self.relu69_1_18_(batchnorm69_1_18_)
        conv70_1_18_padding = self.conv70_1_18_padding(relu69_1_18_)
        conv70_1_18_ = self.conv70_1_18_(conv70_1_18_padding)
        batchnorm70_1_18_ = self.batchnorm70_1_18_(conv70_1_18_)
        relu70_1_18_ = self.relu70_1_18_(batchnorm70_1_18_)
        conv71_1_18_ = self.conv71_1_18_(relu70_1_18_)
        batchnorm71_1_18_ = self.batchnorm71_1_18_(conv71_1_18_)
        conv69_1_19_ = self.conv69_1_19_(relu67_)
        batchnorm69_1_19_ = self.batchnorm69_1_19_(conv69_1_19_)
        relu69_1_19_ = self.relu69_1_19_(batchnorm69_1_19_)
        conv70_1_19_padding = self.conv70_1_19_padding(relu69_1_19_)
        conv70_1_19_ = self.conv70_1_19_(conv70_1_19_padding)
        batchnorm70_1_19_ = self.batchnorm70_1_19_(conv70_1_19_)
        relu70_1_19_ = self.relu70_1_19_(batchnorm70_1_19_)
        conv71_1_19_ = self.conv71_1_19_(relu70_1_19_)
        batchnorm71_1_19_ = self.batchnorm71_1_19_(conv71_1_19_)
        conv69_1_20_ = self.conv69_1_20_(relu67_)
        batchnorm69_1_20_ = self.batchnorm69_1_20_(conv69_1_20_)
        relu69_1_20_ = self.relu69_1_20_(batchnorm69_1_20_)
        conv70_1_20_padding = self.conv70_1_20_padding(relu69_1_20_)
        conv70_1_20_ = self.conv70_1_20_(conv70_1_20_padding)
        batchnorm70_1_20_ = self.batchnorm70_1_20_(conv70_1_20_)
        relu70_1_20_ = self.relu70_1_20_(batchnorm70_1_20_)
        conv71_1_20_ = self.conv71_1_20_(relu70_1_20_)
        batchnorm71_1_20_ = self.batchnorm71_1_20_(conv71_1_20_)
        conv69_1_21_ = self.conv69_1_21_(relu67_)
        batchnorm69_1_21_ = self.batchnorm69_1_21_(conv69_1_21_)
        relu69_1_21_ = self.relu69_1_21_(batchnorm69_1_21_)
        conv70_1_21_padding = self.conv70_1_21_padding(relu69_1_21_)
        conv70_1_21_ = self.conv70_1_21_(conv70_1_21_padding)
        batchnorm70_1_21_ = self.batchnorm70_1_21_(conv70_1_21_)
        relu70_1_21_ = self.relu70_1_21_(batchnorm70_1_21_)
        conv71_1_21_ = self.conv71_1_21_(relu70_1_21_)
        batchnorm71_1_21_ = self.batchnorm71_1_21_(conv71_1_21_)
        conv69_1_22_ = self.conv69_1_22_(relu67_)
        batchnorm69_1_22_ = self.batchnorm69_1_22_(conv69_1_22_)
        relu69_1_22_ = self.relu69_1_22_(batchnorm69_1_22_)
        conv70_1_22_padding = self.conv70_1_22_padding(relu69_1_22_)
        conv70_1_22_ = self.conv70_1_22_(conv70_1_22_padding)
        batchnorm70_1_22_ = self.batchnorm70_1_22_(conv70_1_22_)
        relu70_1_22_ = self.relu70_1_22_(batchnorm70_1_22_)
        conv71_1_22_ = self.conv71_1_22_(relu70_1_22_)
        batchnorm71_1_22_ = self.batchnorm71_1_22_(conv71_1_22_)
        conv69_1_23_ = self.conv69_1_23_(relu67_)
        batchnorm69_1_23_ = self.batchnorm69_1_23_(conv69_1_23_)
        relu69_1_23_ = self.relu69_1_23_(batchnorm69_1_23_)
        conv70_1_23_padding = self.conv70_1_23_padding(relu69_1_23_)
        conv70_1_23_ = self.conv70_1_23_(conv70_1_23_padding)
        batchnorm70_1_23_ = self.batchnorm70_1_23_(conv70_1_23_)
        relu70_1_23_ = self.relu70_1_23_(batchnorm70_1_23_)
        conv71_1_23_ = self.conv71_1_23_(relu70_1_23_)
        batchnorm71_1_23_ = self.batchnorm71_1_23_(conv71_1_23_)
        conv69_1_24_ = self.conv69_1_24_(relu67_)
        batchnorm69_1_24_ = self.batchnorm69_1_24_(conv69_1_24_)
        relu69_1_24_ = self.relu69_1_24_(batchnorm69_1_24_)
        conv70_1_24_padding = self.conv70_1_24_padding(relu69_1_24_)
        conv70_1_24_ = self.conv70_1_24_(conv70_1_24_padding)
        batchnorm70_1_24_ = self.batchnorm70_1_24_(conv70_1_24_)
        relu70_1_24_ = self.relu70_1_24_(batchnorm70_1_24_)
        conv71_1_24_ = self.conv71_1_24_(relu70_1_24_)
        batchnorm71_1_24_ = self.batchnorm71_1_24_(conv71_1_24_)
        conv69_1_25_ = self.conv69_1_25_(relu67_)
        batchnorm69_1_25_ = self.batchnorm69_1_25_(conv69_1_25_)
        relu69_1_25_ = self.relu69_1_25_(batchnorm69_1_25_)
        conv70_1_25_padding = self.conv70_1_25_padding(relu69_1_25_)
        conv70_1_25_ = self.conv70_1_25_(conv70_1_25_padding)
        batchnorm70_1_25_ = self.batchnorm70_1_25_(conv70_1_25_)
        relu70_1_25_ = self.relu70_1_25_(batchnorm70_1_25_)
        conv71_1_25_ = self.conv71_1_25_(relu70_1_25_)
        batchnorm71_1_25_ = self.batchnorm71_1_25_(conv71_1_25_)
        conv69_1_26_ = self.conv69_1_26_(relu67_)
        batchnorm69_1_26_ = self.batchnorm69_1_26_(conv69_1_26_)
        relu69_1_26_ = self.relu69_1_26_(batchnorm69_1_26_)
        conv70_1_26_padding = self.conv70_1_26_padding(relu69_1_26_)
        conv70_1_26_ = self.conv70_1_26_(conv70_1_26_padding)
        batchnorm70_1_26_ = self.batchnorm70_1_26_(conv70_1_26_)
        relu70_1_26_ = self.relu70_1_26_(batchnorm70_1_26_)
        conv71_1_26_ = self.conv71_1_26_(relu70_1_26_)
        batchnorm71_1_26_ = self.batchnorm71_1_26_(conv71_1_26_)
        conv69_1_27_ = self.conv69_1_27_(relu67_)
        batchnorm69_1_27_ = self.batchnorm69_1_27_(conv69_1_27_)
        relu69_1_27_ = self.relu69_1_27_(batchnorm69_1_27_)
        conv70_1_27_padding = self.conv70_1_27_padding(relu69_1_27_)
        conv70_1_27_ = self.conv70_1_27_(conv70_1_27_padding)
        batchnorm70_1_27_ = self.batchnorm70_1_27_(conv70_1_27_)
        relu70_1_27_ = self.relu70_1_27_(batchnorm70_1_27_)
        conv71_1_27_ = self.conv71_1_27_(relu70_1_27_)
        batchnorm71_1_27_ = self.batchnorm71_1_27_(conv71_1_27_)
        conv69_1_28_ = self.conv69_1_28_(relu67_)
        batchnorm69_1_28_ = self.batchnorm69_1_28_(conv69_1_28_)
        relu69_1_28_ = self.relu69_1_28_(batchnorm69_1_28_)
        conv70_1_28_padding = self.conv70_1_28_padding(relu69_1_28_)
        conv70_1_28_ = self.conv70_1_28_(conv70_1_28_padding)
        batchnorm70_1_28_ = self.batchnorm70_1_28_(conv70_1_28_)
        relu70_1_28_ = self.relu70_1_28_(batchnorm70_1_28_)
        conv71_1_28_ = self.conv71_1_28_(relu70_1_28_)
        batchnorm71_1_28_ = self.batchnorm71_1_28_(conv71_1_28_)
        conv69_1_29_ = self.conv69_1_29_(relu67_)
        batchnorm69_1_29_ = self.batchnorm69_1_29_(conv69_1_29_)
        relu69_1_29_ = self.relu69_1_29_(batchnorm69_1_29_)
        conv70_1_29_padding = self.conv70_1_29_padding(relu69_1_29_)
        conv70_1_29_ = self.conv70_1_29_(conv70_1_29_padding)
        batchnorm70_1_29_ = self.batchnorm70_1_29_(conv70_1_29_)
        relu70_1_29_ = self.relu70_1_29_(batchnorm70_1_29_)
        conv71_1_29_ = self.conv71_1_29_(relu70_1_29_)
        batchnorm71_1_29_ = self.batchnorm71_1_29_(conv71_1_29_)
        conv69_1_30_ = self.conv69_1_30_(relu67_)
        batchnorm69_1_30_ = self.batchnorm69_1_30_(conv69_1_30_)
        relu69_1_30_ = self.relu69_1_30_(batchnorm69_1_30_)
        conv70_1_30_padding = self.conv70_1_30_padding(relu69_1_30_)
        conv70_1_30_ = self.conv70_1_30_(conv70_1_30_padding)
        batchnorm70_1_30_ = self.batchnorm70_1_30_(conv70_1_30_)
        relu70_1_30_ = self.relu70_1_30_(batchnorm70_1_30_)
        conv71_1_30_ = self.conv71_1_30_(relu70_1_30_)
        batchnorm71_1_30_ = self.batchnorm71_1_30_(conv71_1_30_)
        conv69_1_31_ = self.conv69_1_31_(relu67_)
        batchnorm69_1_31_ = self.batchnorm69_1_31_(conv69_1_31_)
        relu69_1_31_ = self.relu69_1_31_(batchnorm69_1_31_)
        conv70_1_31_padding = self.conv70_1_31_padding(relu69_1_31_)
        conv70_1_31_ = self.conv70_1_31_(conv70_1_31_padding)
        batchnorm70_1_31_ = self.batchnorm70_1_31_(conv70_1_31_)
        relu70_1_31_ = self.relu70_1_31_(batchnorm70_1_31_)
        conv71_1_31_ = self.conv71_1_31_(relu70_1_31_)
        batchnorm71_1_31_ = self.batchnorm71_1_31_(conv71_1_31_)
        conv69_1_32_ = self.conv69_1_32_(relu67_)
        batchnorm69_1_32_ = self.batchnorm69_1_32_(conv69_1_32_)
        relu69_1_32_ = self.relu69_1_32_(batchnorm69_1_32_)
        conv70_1_32_padding = self.conv70_1_32_padding(relu69_1_32_)
        conv70_1_32_ = self.conv70_1_32_(conv70_1_32_padding)
        batchnorm70_1_32_ = self.batchnorm70_1_32_(conv70_1_32_)
        relu70_1_32_ = self.relu70_1_32_(batchnorm70_1_32_)
        conv71_1_32_ = self.conv71_1_32_(relu70_1_32_)
        batchnorm71_1_32_ = self.batchnorm71_1_32_(conv71_1_32_)
        add72_1_ = batchnorm71_1_1_ + batchnorm71_1_2_ + batchnorm71_1_3_ + batchnorm71_1_4_ + batchnorm71_1_5_ + batchnorm71_1_6_ + batchnorm71_1_7_ + batchnorm71_1_8_ + batchnorm71_1_9_ + batchnorm71_1_10_ + batchnorm71_1_11_ + batchnorm71_1_12_ + batchnorm71_1_13_ + batchnorm71_1_14_ + batchnorm71_1_15_ + batchnorm71_1_16_ + batchnorm71_1_17_ + batchnorm71_1_18_ + batchnorm71_1_19_ + batchnorm71_1_20_ + batchnorm71_1_21_ + batchnorm71_1_22_ + batchnorm71_1_23_ + batchnorm71_1_24_ + batchnorm71_1_25_ + batchnorm71_1_26_ + batchnorm71_1_27_ + batchnorm71_1_28_ + batchnorm71_1_29_ + batchnorm71_1_30_ + batchnorm71_1_31_ + batchnorm71_1_32_
        add73_ = add72_1_ + relu67_
        relu73_ = self.relu73_(add73_)
        conv75_1_1_ = self.conv75_1_1_(relu73_)
        batchnorm75_1_1_ = self.batchnorm75_1_1_(conv75_1_1_)
        relu75_1_1_ = self.relu75_1_1_(batchnorm75_1_1_)
        conv76_1_1_padding = self.conv76_1_1_padding(relu75_1_1_)
        conv76_1_1_ = self.conv76_1_1_(conv76_1_1_padding)
        batchnorm76_1_1_ = self.batchnorm76_1_1_(conv76_1_1_)
        relu76_1_1_ = self.relu76_1_1_(batchnorm76_1_1_)
        conv77_1_1_ = self.conv77_1_1_(relu76_1_1_)
        batchnorm77_1_1_ = self.batchnorm77_1_1_(conv77_1_1_)
        conv75_1_2_ = self.conv75_1_2_(relu73_)
        batchnorm75_1_2_ = self.batchnorm75_1_2_(conv75_1_2_)
        relu75_1_2_ = self.relu75_1_2_(batchnorm75_1_2_)
        conv76_1_2_padding = self.conv76_1_2_padding(relu75_1_2_)
        conv76_1_2_ = self.conv76_1_2_(conv76_1_2_padding)
        batchnorm76_1_2_ = self.batchnorm76_1_2_(conv76_1_2_)
        relu76_1_2_ = self.relu76_1_2_(batchnorm76_1_2_)
        conv77_1_2_ = self.conv77_1_2_(relu76_1_2_)
        batchnorm77_1_2_ = self.batchnorm77_1_2_(conv77_1_2_)
        conv75_1_3_ = self.conv75_1_3_(relu73_)
        batchnorm75_1_3_ = self.batchnorm75_1_3_(conv75_1_3_)
        relu75_1_3_ = self.relu75_1_3_(batchnorm75_1_3_)
        conv76_1_3_padding = self.conv76_1_3_padding(relu75_1_3_)
        conv76_1_3_ = self.conv76_1_3_(conv76_1_3_padding)
        batchnorm76_1_3_ = self.batchnorm76_1_3_(conv76_1_3_)
        relu76_1_3_ = self.relu76_1_3_(batchnorm76_1_3_)
        conv77_1_3_ = self.conv77_1_3_(relu76_1_3_)
        batchnorm77_1_3_ = self.batchnorm77_1_3_(conv77_1_3_)
        conv75_1_4_ = self.conv75_1_4_(relu73_)
        batchnorm75_1_4_ = self.batchnorm75_1_4_(conv75_1_4_)
        relu75_1_4_ = self.relu75_1_4_(batchnorm75_1_4_)
        conv76_1_4_padding = self.conv76_1_4_padding(relu75_1_4_)
        conv76_1_4_ = self.conv76_1_4_(conv76_1_4_padding)
        batchnorm76_1_4_ = self.batchnorm76_1_4_(conv76_1_4_)
        relu76_1_4_ = self.relu76_1_4_(batchnorm76_1_4_)
        conv77_1_4_ = self.conv77_1_4_(relu76_1_4_)
        batchnorm77_1_4_ = self.batchnorm77_1_4_(conv77_1_4_)
        conv75_1_5_ = self.conv75_1_5_(relu73_)
        batchnorm75_1_5_ = self.batchnorm75_1_5_(conv75_1_5_)
        relu75_1_5_ = self.relu75_1_5_(batchnorm75_1_5_)
        conv76_1_5_padding = self.conv76_1_5_padding(relu75_1_5_)
        conv76_1_5_ = self.conv76_1_5_(conv76_1_5_padding)
        batchnorm76_1_5_ = self.batchnorm76_1_5_(conv76_1_5_)
        relu76_1_5_ = self.relu76_1_5_(batchnorm76_1_5_)
        conv77_1_5_ = self.conv77_1_5_(relu76_1_5_)
        batchnorm77_1_5_ = self.batchnorm77_1_5_(conv77_1_5_)
        conv75_1_6_ = self.conv75_1_6_(relu73_)
        batchnorm75_1_6_ = self.batchnorm75_1_6_(conv75_1_6_)
        relu75_1_6_ = self.relu75_1_6_(batchnorm75_1_6_)
        conv76_1_6_padding = self.conv76_1_6_padding(relu75_1_6_)
        conv76_1_6_ = self.conv76_1_6_(conv76_1_6_padding)
        batchnorm76_1_6_ = self.batchnorm76_1_6_(conv76_1_6_)
        relu76_1_6_ = self.relu76_1_6_(batchnorm76_1_6_)
        conv77_1_6_ = self.conv77_1_6_(relu76_1_6_)
        batchnorm77_1_6_ = self.batchnorm77_1_6_(conv77_1_6_)
        conv75_1_7_ = self.conv75_1_7_(relu73_)
        batchnorm75_1_7_ = self.batchnorm75_1_7_(conv75_1_7_)
        relu75_1_7_ = self.relu75_1_7_(batchnorm75_1_7_)
        conv76_1_7_padding = self.conv76_1_7_padding(relu75_1_7_)
        conv76_1_7_ = self.conv76_1_7_(conv76_1_7_padding)
        batchnorm76_1_7_ = self.batchnorm76_1_7_(conv76_1_7_)
        relu76_1_7_ = self.relu76_1_7_(batchnorm76_1_7_)
        conv77_1_7_ = self.conv77_1_7_(relu76_1_7_)
        batchnorm77_1_7_ = self.batchnorm77_1_7_(conv77_1_7_)
        conv75_1_8_ = self.conv75_1_8_(relu73_)
        batchnorm75_1_8_ = self.batchnorm75_1_8_(conv75_1_8_)
        relu75_1_8_ = self.relu75_1_8_(batchnorm75_1_8_)
        conv76_1_8_padding = self.conv76_1_8_padding(relu75_1_8_)
        conv76_1_8_ = self.conv76_1_8_(conv76_1_8_padding)
        batchnorm76_1_8_ = self.batchnorm76_1_8_(conv76_1_8_)
        relu76_1_8_ = self.relu76_1_8_(batchnorm76_1_8_)
        conv77_1_8_ = self.conv77_1_8_(relu76_1_8_)
        batchnorm77_1_8_ = self.batchnorm77_1_8_(conv77_1_8_)
        conv75_1_9_ = self.conv75_1_9_(relu73_)
        batchnorm75_1_9_ = self.batchnorm75_1_9_(conv75_1_9_)
        relu75_1_9_ = self.relu75_1_9_(batchnorm75_1_9_)
        conv76_1_9_padding = self.conv76_1_9_padding(relu75_1_9_)
        conv76_1_9_ = self.conv76_1_9_(conv76_1_9_padding)
        batchnorm76_1_9_ = self.batchnorm76_1_9_(conv76_1_9_)
        relu76_1_9_ = self.relu76_1_9_(batchnorm76_1_9_)
        conv77_1_9_ = self.conv77_1_9_(relu76_1_9_)
        batchnorm77_1_9_ = self.batchnorm77_1_9_(conv77_1_9_)
        conv75_1_10_ = self.conv75_1_10_(relu73_)
        batchnorm75_1_10_ = self.batchnorm75_1_10_(conv75_1_10_)
        relu75_1_10_ = self.relu75_1_10_(batchnorm75_1_10_)
        conv76_1_10_padding = self.conv76_1_10_padding(relu75_1_10_)
        conv76_1_10_ = self.conv76_1_10_(conv76_1_10_padding)
        batchnorm76_1_10_ = self.batchnorm76_1_10_(conv76_1_10_)
        relu76_1_10_ = self.relu76_1_10_(batchnorm76_1_10_)
        conv77_1_10_ = self.conv77_1_10_(relu76_1_10_)
        batchnorm77_1_10_ = self.batchnorm77_1_10_(conv77_1_10_)
        conv75_1_11_ = self.conv75_1_11_(relu73_)
        batchnorm75_1_11_ = self.batchnorm75_1_11_(conv75_1_11_)
        relu75_1_11_ = self.relu75_1_11_(batchnorm75_1_11_)
        conv76_1_11_padding = self.conv76_1_11_padding(relu75_1_11_)
        conv76_1_11_ = self.conv76_1_11_(conv76_1_11_padding)
        batchnorm76_1_11_ = self.batchnorm76_1_11_(conv76_1_11_)
        relu76_1_11_ = self.relu76_1_11_(batchnorm76_1_11_)
        conv77_1_11_ = self.conv77_1_11_(relu76_1_11_)
        batchnorm77_1_11_ = self.batchnorm77_1_11_(conv77_1_11_)
        conv75_1_12_ = self.conv75_1_12_(relu73_)
        batchnorm75_1_12_ = self.batchnorm75_1_12_(conv75_1_12_)
        relu75_1_12_ = self.relu75_1_12_(batchnorm75_1_12_)
        conv76_1_12_padding = self.conv76_1_12_padding(relu75_1_12_)
        conv76_1_12_ = self.conv76_1_12_(conv76_1_12_padding)
        batchnorm76_1_12_ = self.batchnorm76_1_12_(conv76_1_12_)
        relu76_1_12_ = self.relu76_1_12_(batchnorm76_1_12_)
        conv77_1_12_ = self.conv77_1_12_(relu76_1_12_)
        batchnorm77_1_12_ = self.batchnorm77_1_12_(conv77_1_12_)
        conv75_1_13_ = self.conv75_1_13_(relu73_)
        batchnorm75_1_13_ = self.batchnorm75_1_13_(conv75_1_13_)
        relu75_1_13_ = self.relu75_1_13_(batchnorm75_1_13_)
        conv76_1_13_padding = self.conv76_1_13_padding(relu75_1_13_)
        conv76_1_13_ = self.conv76_1_13_(conv76_1_13_padding)
        batchnorm76_1_13_ = self.batchnorm76_1_13_(conv76_1_13_)
        relu76_1_13_ = self.relu76_1_13_(batchnorm76_1_13_)
        conv77_1_13_ = self.conv77_1_13_(relu76_1_13_)
        batchnorm77_1_13_ = self.batchnorm77_1_13_(conv77_1_13_)
        conv75_1_14_ = self.conv75_1_14_(relu73_)
        batchnorm75_1_14_ = self.batchnorm75_1_14_(conv75_1_14_)
        relu75_1_14_ = self.relu75_1_14_(batchnorm75_1_14_)
        conv76_1_14_padding = self.conv76_1_14_padding(relu75_1_14_)
        conv76_1_14_ = self.conv76_1_14_(conv76_1_14_padding)
        batchnorm76_1_14_ = self.batchnorm76_1_14_(conv76_1_14_)
        relu76_1_14_ = self.relu76_1_14_(batchnorm76_1_14_)
        conv77_1_14_ = self.conv77_1_14_(relu76_1_14_)
        batchnorm77_1_14_ = self.batchnorm77_1_14_(conv77_1_14_)
        conv75_1_15_ = self.conv75_1_15_(relu73_)
        batchnorm75_1_15_ = self.batchnorm75_1_15_(conv75_1_15_)
        relu75_1_15_ = self.relu75_1_15_(batchnorm75_1_15_)
        conv76_1_15_padding = self.conv76_1_15_padding(relu75_1_15_)
        conv76_1_15_ = self.conv76_1_15_(conv76_1_15_padding)
        batchnorm76_1_15_ = self.batchnorm76_1_15_(conv76_1_15_)
        relu76_1_15_ = self.relu76_1_15_(batchnorm76_1_15_)
        conv77_1_15_ = self.conv77_1_15_(relu76_1_15_)
        batchnorm77_1_15_ = self.batchnorm77_1_15_(conv77_1_15_)
        conv75_1_16_ = self.conv75_1_16_(relu73_)
        batchnorm75_1_16_ = self.batchnorm75_1_16_(conv75_1_16_)
        relu75_1_16_ = self.relu75_1_16_(batchnorm75_1_16_)
        conv76_1_16_padding = self.conv76_1_16_padding(relu75_1_16_)
        conv76_1_16_ = self.conv76_1_16_(conv76_1_16_padding)
        batchnorm76_1_16_ = self.batchnorm76_1_16_(conv76_1_16_)
        relu76_1_16_ = self.relu76_1_16_(batchnorm76_1_16_)
        conv77_1_16_ = self.conv77_1_16_(relu76_1_16_)
        batchnorm77_1_16_ = self.batchnorm77_1_16_(conv77_1_16_)
        conv75_1_17_ = self.conv75_1_17_(relu73_)
        batchnorm75_1_17_ = self.batchnorm75_1_17_(conv75_1_17_)
        relu75_1_17_ = self.relu75_1_17_(batchnorm75_1_17_)
        conv76_1_17_padding = self.conv76_1_17_padding(relu75_1_17_)
        conv76_1_17_ = self.conv76_1_17_(conv76_1_17_padding)
        batchnorm76_1_17_ = self.batchnorm76_1_17_(conv76_1_17_)
        relu76_1_17_ = self.relu76_1_17_(batchnorm76_1_17_)
        conv77_1_17_ = self.conv77_1_17_(relu76_1_17_)
        batchnorm77_1_17_ = self.batchnorm77_1_17_(conv77_1_17_)
        conv75_1_18_ = self.conv75_1_18_(relu73_)
        batchnorm75_1_18_ = self.batchnorm75_1_18_(conv75_1_18_)
        relu75_1_18_ = self.relu75_1_18_(batchnorm75_1_18_)
        conv76_1_18_padding = self.conv76_1_18_padding(relu75_1_18_)
        conv76_1_18_ = self.conv76_1_18_(conv76_1_18_padding)
        batchnorm76_1_18_ = self.batchnorm76_1_18_(conv76_1_18_)
        relu76_1_18_ = self.relu76_1_18_(batchnorm76_1_18_)
        conv77_1_18_ = self.conv77_1_18_(relu76_1_18_)
        batchnorm77_1_18_ = self.batchnorm77_1_18_(conv77_1_18_)
        conv75_1_19_ = self.conv75_1_19_(relu73_)
        batchnorm75_1_19_ = self.batchnorm75_1_19_(conv75_1_19_)
        relu75_1_19_ = self.relu75_1_19_(batchnorm75_1_19_)
        conv76_1_19_padding = self.conv76_1_19_padding(relu75_1_19_)
        conv76_1_19_ = self.conv76_1_19_(conv76_1_19_padding)
        batchnorm76_1_19_ = self.batchnorm76_1_19_(conv76_1_19_)
        relu76_1_19_ = self.relu76_1_19_(batchnorm76_1_19_)
        conv77_1_19_ = self.conv77_1_19_(relu76_1_19_)
        batchnorm77_1_19_ = self.batchnorm77_1_19_(conv77_1_19_)
        conv75_1_20_ = self.conv75_1_20_(relu73_)
        batchnorm75_1_20_ = self.batchnorm75_1_20_(conv75_1_20_)
        relu75_1_20_ = self.relu75_1_20_(batchnorm75_1_20_)
        conv76_1_20_padding = self.conv76_1_20_padding(relu75_1_20_)
        conv76_1_20_ = self.conv76_1_20_(conv76_1_20_padding)
        batchnorm76_1_20_ = self.batchnorm76_1_20_(conv76_1_20_)
        relu76_1_20_ = self.relu76_1_20_(batchnorm76_1_20_)
        conv77_1_20_ = self.conv77_1_20_(relu76_1_20_)
        batchnorm77_1_20_ = self.batchnorm77_1_20_(conv77_1_20_)
        conv75_1_21_ = self.conv75_1_21_(relu73_)
        batchnorm75_1_21_ = self.batchnorm75_1_21_(conv75_1_21_)
        relu75_1_21_ = self.relu75_1_21_(batchnorm75_1_21_)
        conv76_1_21_padding = self.conv76_1_21_padding(relu75_1_21_)
        conv76_1_21_ = self.conv76_1_21_(conv76_1_21_padding)
        batchnorm76_1_21_ = self.batchnorm76_1_21_(conv76_1_21_)
        relu76_1_21_ = self.relu76_1_21_(batchnorm76_1_21_)
        conv77_1_21_ = self.conv77_1_21_(relu76_1_21_)
        batchnorm77_1_21_ = self.batchnorm77_1_21_(conv77_1_21_)
        conv75_1_22_ = self.conv75_1_22_(relu73_)
        batchnorm75_1_22_ = self.batchnorm75_1_22_(conv75_1_22_)
        relu75_1_22_ = self.relu75_1_22_(batchnorm75_1_22_)
        conv76_1_22_padding = self.conv76_1_22_padding(relu75_1_22_)
        conv76_1_22_ = self.conv76_1_22_(conv76_1_22_padding)
        batchnorm76_1_22_ = self.batchnorm76_1_22_(conv76_1_22_)
        relu76_1_22_ = self.relu76_1_22_(batchnorm76_1_22_)
        conv77_1_22_ = self.conv77_1_22_(relu76_1_22_)
        batchnorm77_1_22_ = self.batchnorm77_1_22_(conv77_1_22_)
        conv75_1_23_ = self.conv75_1_23_(relu73_)
        batchnorm75_1_23_ = self.batchnorm75_1_23_(conv75_1_23_)
        relu75_1_23_ = self.relu75_1_23_(batchnorm75_1_23_)
        conv76_1_23_padding = self.conv76_1_23_padding(relu75_1_23_)
        conv76_1_23_ = self.conv76_1_23_(conv76_1_23_padding)
        batchnorm76_1_23_ = self.batchnorm76_1_23_(conv76_1_23_)
        relu76_1_23_ = self.relu76_1_23_(batchnorm76_1_23_)
        conv77_1_23_ = self.conv77_1_23_(relu76_1_23_)
        batchnorm77_1_23_ = self.batchnorm77_1_23_(conv77_1_23_)
        conv75_1_24_ = self.conv75_1_24_(relu73_)
        batchnorm75_1_24_ = self.batchnorm75_1_24_(conv75_1_24_)
        relu75_1_24_ = self.relu75_1_24_(batchnorm75_1_24_)
        conv76_1_24_padding = self.conv76_1_24_padding(relu75_1_24_)
        conv76_1_24_ = self.conv76_1_24_(conv76_1_24_padding)
        batchnorm76_1_24_ = self.batchnorm76_1_24_(conv76_1_24_)
        relu76_1_24_ = self.relu76_1_24_(batchnorm76_1_24_)
        conv77_1_24_ = self.conv77_1_24_(relu76_1_24_)
        batchnorm77_1_24_ = self.batchnorm77_1_24_(conv77_1_24_)
        conv75_1_25_ = self.conv75_1_25_(relu73_)
        batchnorm75_1_25_ = self.batchnorm75_1_25_(conv75_1_25_)
        relu75_1_25_ = self.relu75_1_25_(batchnorm75_1_25_)
        conv76_1_25_padding = self.conv76_1_25_padding(relu75_1_25_)
        conv76_1_25_ = self.conv76_1_25_(conv76_1_25_padding)
        batchnorm76_1_25_ = self.batchnorm76_1_25_(conv76_1_25_)
        relu76_1_25_ = self.relu76_1_25_(batchnorm76_1_25_)
        conv77_1_25_ = self.conv77_1_25_(relu76_1_25_)
        batchnorm77_1_25_ = self.batchnorm77_1_25_(conv77_1_25_)
        conv75_1_26_ = self.conv75_1_26_(relu73_)
        batchnorm75_1_26_ = self.batchnorm75_1_26_(conv75_1_26_)
        relu75_1_26_ = self.relu75_1_26_(batchnorm75_1_26_)
        conv76_1_26_padding = self.conv76_1_26_padding(relu75_1_26_)
        conv76_1_26_ = self.conv76_1_26_(conv76_1_26_padding)
        batchnorm76_1_26_ = self.batchnorm76_1_26_(conv76_1_26_)
        relu76_1_26_ = self.relu76_1_26_(batchnorm76_1_26_)
        conv77_1_26_ = self.conv77_1_26_(relu76_1_26_)
        batchnorm77_1_26_ = self.batchnorm77_1_26_(conv77_1_26_)
        conv75_1_27_ = self.conv75_1_27_(relu73_)
        batchnorm75_1_27_ = self.batchnorm75_1_27_(conv75_1_27_)
        relu75_1_27_ = self.relu75_1_27_(batchnorm75_1_27_)
        conv76_1_27_padding = self.conv76_1_27_padding(relu75_1_27_)
        conv76_1_27_ = self.conv76_1_27_(conv76_1_27_padding)
        batchnorm76_1_27_ = self.batchnorm76_1_27_(conv76_1_27_)
        relu76_1_27_ = self.relu76_1_27_(batchnorm76_1_27_)
        conv77_1_27_ = self.conv77_1_27_(relu76_1_27_)
        batchnorm77_1_27_ = self.batchnorm77_1_27_(conv77_1_27_)
        conv75_1_28_ = self.conv75_1_28_(relu73_)
        batchnorm75_1_28_ = self.batchnorm75_1_28_(conv75_1_28_)
        relu75_1_28_ = self.relu75_1_28_(batchnorm75_1_28_)
        conv76_1_28_padding = self.conv76_1_28_padding(relu75_1_28_)
        conv76_1_28_ = self.conv76_1_28_(conv76_1_28_padding)
        batchnorm76_1_28_ = self.batchnorm76_1_28_(conv76_1_28_)
        relu76_1_28_ = self.relu76_1_28_(batchnorm76_1_28_)
        conv77_1_28_ = self.conv77_1_28_(relu76_1_28_)
        batchnorm77_1_28_ = self.batchnorm77_1_28_(conv77_1_28_)
        conv75_1_29_ = self.conv75_1_29_(relu73_)
        batchnorm75_1_29_ = self.batchnorm75_1_29_(conv75_1_29_)
        relu75_1_29_ = self.relu75_1_29_(batchnorm75_1_29_)
        conv76_1_29_padding = self.conv76_1_29_padding(relu75_1_29_)
        conv76_1_29_ = self.conv76_1_29_(conv76_1_29_padding)
        batchnorm76_1_29_ = self.batchnorm76_1_29_(conv76_1_29_)
        relu76_1_29_ = self.relu76_1_29_(batchnorm76_1_29_)
        conv77_1_29_ = self.conv77_1_29_(relu76_1_29_)
        batchnorm77_1_29_ = self.batchnorm77_1_29_(conv77_1_29_)
        conv75_1_30_ = self.conv75_1_30_(relu73_)
        batchnorm75_1_30_ = self.batchnorm75_1_30_(conv75_1_30_)
        relu75_1_30_ = self.relu75_1_30_(batchnorm75_1_30_)
        conv76_1_30_padding = self.conv76_1_30_padding(relu75_1_30_)
        conv76_1_30_ = self.conv76_1_30_(conv76_1_30_padding)
        batchnorm76_1_30_ = self.batchnorm76_1_30_(conv76_1_30_)
        relu76_1_30_ = self.relu76_1_30_(batchnorm76_1_30_)
        conv77_1_30_ = self.conv77_1_30_(relu76_1_30_)
        batchnorm77_1_30_ = self.batchnorm77_1_30_(conv77_1_30_)
        conv75_1_31_ = self.conv75_1_31_(relu73_)
        batchnorm75_1_31_ = self.batchnorm75_1_31_(conv75_1_31_)
        relu75_1_31_ = self.relu75_1_31_(batchnorm75_1_31_)
        conv76_1_31_padding = self.conv76_1_31_padding(relu75_1_31_)
        conv76_1_31_ = self.conv76_1_31_(conv76_1_31_padding)
        batchnorm76_1_31_ = self.batchnorm76_1_31_(conv76_1_31_)
        relu76_1_31_ = self.relu76_1_31_(batchnorm76_1_31_)
        conv77_1_31_ = self.conv77_1_31_(relu76_1_31_)
        batchnorm77_1_31_ = self.batchnorm77_1_31_(conv77_1_31_)
        conv75_1_32_ = self.conv75_1_32_(relu73_)
        batchnorm75_1_32_ = self.batchnorm75_1_32_(conv75_1_32_)
        relu75_1_32_ = self.relu75_1_32_(batchnorm75_1_32_)
        conv76_1_32_padding = self.conv76_1_32_padding(relu75_1_32_)
        conv76_1_32_ = self.conv76_1_32_(conv76_1_32_padding)
        batchnorm76_1_32_ = self.batchnorm76_1_32_(conv76_1_32_)
        relu76_1_32_ = self.relu76_1_32_(batchnorm76_1_32_)
        conv77_1_32_ = self.conv77_1_32_(relu76_1_32_)
        batchnorm77_1_32_ = self.batchnorm77_1_32_(conv77_1_32_)
        add78_1_ = batchnorm77_1_1_ + batchnorm77_1_2_ + batchnorm77_1_3_ + batchnorm77_1_4_ + batchnorm77_1_5_ + batchnorm77_1_6_ + batchnorm77_1_7_ + batchnorm77_1_8_ + batchnorm77_1_9_ + batchnorm77_1_10_ + batchnorm77_1_11_ + batchnorm77_1_12_ + batchnorm77_1_13_ + batchnorm77_1_14_ + batchnorm77_1_15_ + batchnorm77_1_16_ + batchnorm77_1_17_ + batchnorm77_1_18_ + batchnorm77_1_19_ + batchnorm77_1_20_ + batchnorm77_1_21_ + batchnorm77_1_22_ + batchnorm77_1_23_ + batchnorm77_1_24_ + batchnorm77_1_25_ + batchnorm77_1_26_ + batchnorm77_1_27_ + batchnorm77_1_28_ + batchnorm77_1_29_ + batchnorm77_1_30_ + batchnorm77_1_31_ + batchnorm77_1_32_
        add79_ = add78_1_ + relu73_
        relu79_ = self.relu79_(add79_)
        conv81_1_1_ = self.conv81_1_1_(relu79_)
        batchnorm81_1_1_ = self.batchnorm81_1_1_(conv81_1_1_)
        relu81_1_1_ = self.relu81_1_1_(batchnorm81_1_1_)
        conv82_1_1_padding = self.conv82_1_1_padding(relu81_1_1_)
        conv82_1_1_ = self.conv82_1_1_(conv82_1_1_padding)
        batchnorm82_1_1_ = self.batchnorm82_1_1_(conv82_1_1_)
        relu82_1_1_ = self.relu82_1_1_(batchnorm82_1_1_)
        conv83_1_1_ = self.conv83_1_1_(relu82_1_1_)
        batchnorm83_1_1_ = self.batchnorm83_1_1_(conv83_1_1_)
        conv81_1_2_ = self.conv81_1_2_(relu79_)
        batchnorm81_1_2_ = self.batchnorm81_1_2_(conv81_1_2_)
        relu81_1_2_ = self.relu81_1_2_(batchnorm81_1_2_)
        conv82_1_2_padding = self.conv82_1_2_padding(relu81_1_2_)
        conv82_1_2_ = self.conv82_1_2_(conv82_1_2_padding)
        batchnorm82_1_2_ = self.batchnorm82_1_2_(conv82_1_2_)
        relu82_1_2_ = self.relu82_1_2_(batchnorm82_1_2_)
        conv83_1_2_ = self.conv83_1_2_(relu82_1_2_)
        batchnorm83_1_2_ = self.batchnorm83_1_2_(conv83_1_2_)
        conv81_1_3_ = self.conv81_1_3_(relu79_)
        batchnorm81_1_3_ = self.batchnorm81_1_3_(conv81_1_3_)
        relu81_1_3_ = self.relu81_1_3_(batchnorm81_1_3_)
        conv82_1_3_padding = self.conv82_1_3_padding(relu81_1_3_)
        conv82_1_3_ = self.conv82_1_3_(conv82_1_3_padding)
        batchnorm82_1_3_ = self.batchnorm82_1_3_(conv82_1_3_)
        relu82_1_3_ = self.relu82_1_3_(batchnorm82_1_3_)
        conv83_1_3_ = self.conv83_1_3_(relu82_1_3_)
        batchnorm83_1_3_ = self.batchnorm83_1_3_(conv83_1_3_)
        conv81_1_4_ = self.conv81_1_4_(relu79_)
        batchnorm81_1_4_ = self.batchnorm81_1_4_(conv81_1_4_)
        relu81_1_4_ = self.relu81_1_4_(batchnorm81_1_4_)
        conv82_1_4_padding = self.conv82_1_4_padding(relu81_1_4_)
        conv82_1_4_ = self.conv82_1_4_(conv82_1_4_padding)
        batchnorm82_1_4_ = self.batchnorm82_1_4_(conv82_1_4_)
        relu82_1_4_ = self.relu82_1_4_(batchnorm82_1_4_)
        conv83_1_4_ = self.conv83_1_4_(relu82_1_4_)
        batchnorm83_1_4_ = self.batchnorm83_1_4_(conv83_1_4_)
        conv81_1_5_ = self.conv81_1_5_(relu79_)
        batchnorm81_1_5_ = self.batchnorm81_1_5_(conv81_1_5_)
        relu81_1_5_ = self.relu81_1_5_(batchnorm81_1_5_)
        conv82_1_5_padding = self.conv82_1_5_padding(relu81_1_5_)
        conv82_1_5_ = self.conv82_1_5_(conv82_1_5_padding)
        batchnorm82_1_5_ = self.batchnorm82_1_5_(conv82_1_5_)
        relu82_1_5_ = self.relu82_1_5_(batchnorm82_1_5_)
        conv83_1_5_ = self.conv83_1_5_(relu82_1_5_)
        batchnorm83_1_5_ = self.batchnorm83_1_5_(conv83_1_5_)
        conv81_1_6_ = self.conv81_1_6_(relu79_)
        batchnorm81_1_6_ = self.batchnorm81_1_6_(conv81_1_6_)
        relu81_1_6_ = self.relu81_1_6_(batchnorm81_1_6_)
        conv82_1_6_padding = self.conv82_1_6_padding(relu81_1_6_)
        conv82_1_6_ = self.conv82_1_6_(conv82_1_6_padding)
        batchnorm82_1_6_ = self.batchnorm82_1_6_(conv82_1_6_)
        relu82_1_6_ = self.relu82_1_6_(batchnorm82_1_6_)
        conv83_1_6_ = self.conv83_1_6_(relu82_1_6_)
        batchnorm83_1_6_ = self.batchnorm83_1_6_(conv83_1_6_)
        conv81_1_7_ = self.conv81_1_7_(relu79_)
        batchnorm81_1_7_ = self.batchnorm81_1_7_(conv81_1_7_)
        relu81_1_7_ = self.relu81_1_7_(batchnorm81_1_7_)
        conv82_1_7_padding = self.conv82_1_7_padding(relu81_1_7_)
        conv82_1_7_ = self.conv82_1_7_(conv82_1_7_padding)
        batchnorm82_1_7_ = self.batchnorm82_1_7_(conv82_1_7_)
        relu82_1_7_ = self.relu82_1_7_(batchnorm82_1_7_)
        conv83_1_7_ = self.conv83_1_7_(relu82_1_7_)
        batchnorm83_1_7_ = self.batchnorm83_1_7_(conv83_1_7_)
        conv81_1_8_ = self.conv81_1_8_(relu79_)
        batchnorm81_1_8_ = self.batchnorm81_1_8_(conv81_1_8_)
        relu81_1_8_ = self.relu81_1_8_(batchnorm81_1_8_)
        conv82_1_8_padding = self.conv82_1_8_padding(relu81_1_8_)
        conv82_1_8_ = self.conv82_1_8_(conv82_1_8_padding)
        batchnorm82_1_8_ = self.batchnorm82_1_8_(conv82_1_8_)
        relu82_1_8_ = self.relu82_1_8_(batchnorm82_1_8_)
        conv83_1_8_ = self.conv83_1_8_(relu82_1_8_)
        batchnorm83_1_8_ = self.batchnorm83_1_8_(conv83_1_8_)
        conv81_1_9_ = self.conv81_1_9_(relu79_)
        batchnorm81_1_9_ = self.batchnorm81_1_9_(conv81_1_9_)
        relu81_1_9_ = self.relu81_1_9_(batchnorm81_1_9_)
        conv82_1_9_padding = self.conv82_1_9_padding(relu81_1_9_)
        conv82_1_9_ = self.conv82_1_9_(conv82_1_9_padding)
        batchnorm82_1_9_ = self.batchnorm82_1_9_(conv82_1_9_)
        relu82_1_9_ = self.relu82_1_9_(batchnorm82_1_9_)
        conv83_1_9_ = self.conv83_1_9_(relu82_1_9_)
        batchnorm83_1_9_ = self.batchnorm83_1_9_(conv83_1_9_)
        conv81_1_10_ = self.conv81_1_10_(relu79_)
        batchnorm81_1_10_ = self.batchnorm81_1_10_(conv81_1_10_)
        relu81_1_10_ = self.relu81_1_10_(batchnorm81_1_10_)
        conv82_1_10_padding = self.conv82_1_10_padding(relu81_1_10_)
        conv82_1_10_ = self.conv82_1_10_(conv82_1_10_padding)
        batchnorm82_1_10_ = self.batchnorm82_1_10_(conv82_1_10_)
        relu82_1_10_ = self.relu82_1_10_(batchnorm82_1_10_)
        conv83_1_10_ = self.conv83_1_10_(relu82_1_10_)
        batchnorm83_1_10_ = self.batchnorm83_1_10_(conv83_1_10_)
        conv81_1_11_ = self.conv81_1_11_(relu79_)
        batchnorm81_1_11_ = self.batchnorm81_1_11_(conv81_1_11_)
        relu81_1_11_ = self.relu81_1_11_(batchnorm81_1_11_)
        conv82_1_11_padding = self.conv82_1_11_padding(relu81_1_11_)
        conv82_1_11_ = self.conv82_1_11_(conv82_1_11_padding)
        batchnorm82_1_11_ = self.batchnorm82_1_11_(conv82_1_11_)
        relu82_1_11_ = self.relu82_1_11_(batchnorm82_1_11_)
        conv83_1_11_ = self.conv83_1_11_(relu82_1_11_)
        batchnorm83_1_11_ = self.batchnorm83_1_11_(conv83_1_11_)
        conv81_1_12_ = self.conv81_1_12_(relu79_)
        batchnorm81_1_12_ = self.batchnorm81_1_12_(conv81_1_12_)
        relu81_1_12_ = self.relu81_1_12_(batchnorm81_1_12_)
        conv82_1_12_padding = self.conv82_1_12_padding(relu81_1_12_)
        conv82_1_12_ = self.conv82_1_12_(conv82_1_12_padding)
        batchnorm82_1_12_ = self.batchnorm82_1_12_(conv82_1_12_)
        relu82_1_12_ = self.relu82_1_12_(batchnorm82_1_12_)
        conv83_1_12_ = self.conv83_1_12_(relu82_1_12_)
        batchnorm83_1_12_ = self.batchnorm83_1_12_(conv83_1_12_)
        conv81_1_13_ = self.conv81_1_13_(relu79_)
        batchnorm81_1_13_ = self.batchnorm81_1_13_(conv81_1_13_)
        relu81_1_13_ = self.relu81_1_13_(batchnorm81_1_13_)
        conv82_1_13_padding = self.conv82_1_13_padding(relu81_1_13_)
        conv82_1_13_ = self.conv82_1_13_(conv82_1_13_padding)
        batchnorm82_1_13_ = self.batchnorm82_1_13_(conv82_1_13_)
        relu82_1_13_ = self.relu82_1_13_(batchnorm82_1_13_)
        conv83_1_13_ = self.conv83_1_13_(relu82_1_13_)
        batchnorm83_1_13_ = self.batchnorm83_1_13_(conv83_1_13_)
        conv81_1_14_ = self.conv81_1_14_(relu79_)
        batchnorm81_1_14_ = self.batchnorm81_1_14_(conv81_1_14_)
        relu81_1_14_ = self.relu81_1_14_(batchnorm81_1_14_)
        conv82_1_14_padding = self.conv82_1_14_padding(relu81_1_14_)
        conv82_1_14_ = self.conv82_1_14_(conv82_1_14_padding)
        batchnorm82_1_14_ = self.batchnorm82_1_14_(conv82_1_14_)
        relu82_1_14_ = self.relu82_1_14_(batchnorm82_1_14_)
        conv83_1_14_ = self.conv83_1_14_(relu82_1_14_)
        batchnorm83_1_14_ = self.batchnorm83_1_14_(conv83_1_14_)
        conv81_1_15_ = self.conv81_1_15_(relu79_)
        batchnorm81_1_15_ = self.batchnorm81_1_15_(conv81_1_15_)
        relu81_1_15_ = self.relu81_1_15_(batchnorm81_1_15_)
        conv82_1_15_padding = self.conv82_1_15_padding(relu81_1_15_)
        conv82_1_15_ = self.conv82_1_15_(conv82_1_15_padding)
        batchnorm82_1_15_ = self.batchnorm82_1_15_(conv82_1_15_)
        relu82_1_15_ = self.relu82_1_15_(batchnorm82_1_15_)
        conv83_1_15_ = self.conv83_1_15_(relu82_1_15_)
        batchnorm83_1_15_ = self.batchnorm83_1_15_(conv83_1_15_)
        conv81_1_16_ = self.conv81_1_16_(relu79_)
        batchnorm81_1_16_ = self.batchnorm81_1_16_(conv81_1_16_)
        relu81_1_16_ = self.relu81_1_16_(batchnorm81_1_16_)
        conv82_1_16_padding = self.conv82_1_16_padding(relu81_1_16_)
        conv82_1_16_ = self.conv82_1_16_(conv82_1_16_padding)
        batchnorm82_1_16_ = self.batchnorm82_1_16_(conv82_1_16_)
        relu82_1_16_ = self.relu82_1_16_(batchnorm82_1_16_)
        conv83_1_16_ = self.conv83_1_16_(relu82_1_16_)
        batchnorm83_1_16_ = self.batchnorm83_1_16_(conv83_1_16_)
        conv81_1_17_ = self.conv81_1_17_(relu79_)
        batchnorm81_1_17_ = self.batchnorm81_1_17_(conv81_1_17_)
        relu81_1_17_ = self.relu81_1_17_(batchnorm81_1_17_)
        conv82_1_17_padding = self.conv82_1_17_padding(relu81_1_17_)
        conv82_1_17_ = self.conv82_1_17_(conv82_1_17_padding)
        batchnorm82_1_17_ = self.batchnorm82_1_17_(conv82_1_17_)
        relu82_1_17_ = self.relu82_1_17_(batchnorm82_1_17_)
        conv83_1_17_ = self.conv83_1_17_(relu82_1_17_)
        batchnorm83_1_17_ = self.batchnorm83_1_17_(conv83_1_17_)
        conv81_1_18_ = self.conv81_1_18_(relu79_)
        batchnorm81_1_18_ = self.batchnorm81_1_18_(conv81_1_18_)
        relu81_1_18_ = self.relu81_1_18_(batchnorm81_1_18_)
        conv82_1_18_padding = self.conv82_1_18_padding(relu81_1_18_)
        conv82_1_18_ = self.conv82_1_18_(conv82_1_18_padding)
        batchnorm82_1_18_ = self.batchnorm82_1_18_(conv82_1_18_)
        relu82_1_18_ = self.relu82_1_18_(batchnorm82_1_18_)
        conv83_1_18_ = self.conv83_1_18_(relu82_1_18_)
        batchnorm83_1_18_ = self.batchnorm83_1_18_(conv83_1_18_)
        conv81_1_19_ = self.conv81_1_19_(relu79_)
        batchnorm81_1_19_ = self.batchnorm81_1_19_(conv81_1_19_)
        relu81_1_19_ = self.relu81_1_19_(batchnorm81_1_19_)
        conv82_1_19_padding = self.conv82_1_19_padding(relu81_1_19_)
        conv82_1_19_ = self.conv82_1_19_(conv82_1_19_padding)
        batchnorm82_1_19_ = self.batchnorm82_1_19_(conv82_1_19_)
        relu82_1_19_ = self.relu82_1_19_(batchnorm82_1_19_)
        conv83_1_19_ = self.conv83_1_19_(relu82_1_19_)
        batchnorm83_1_19_ = self.batchnorm83_1_19_(conv83_1_19_)
        conv81_1_20_ = self.conv81_1_20_(relu79_)
        batchnorm81_1_20_ = self.batchnorm81_1_20_(conv81_1_20_)
        relu81_1_20_ = self.relu81_1_20_(batchnorm81_1_20_)
        conv82_1_20_padding = self.conv82_1_20_padding(relu81_1_20_)
        conv82_1_20_ = self.conv82_1_20_(conv82_1_20_padding)
        batchnorm82_1_20_ = self.batchnorm82_1_20_(conv82_1_20_)
        relu82_1_20_ = self.relu82_1_20_(batchnorm82_1_20_)
        conv83_1_20_ = self.conv83_1_20_(relu82_1_20_)
        batchnorm83_1_20_ = self.batchnorm83_1_20_(conv83_1_20_)
        conv81_1_21_ = self.conv81_1_21_(relu79_)
        batchnorm81_1_21_ = self.batchnorm81_1_21_(conv81_1_21_)
        relu81_1_21_ = self.relu81_1_21_(batchnorm81_1_21_)
        conv82_1_21_padding = self.conv82_1_21_padding(relu81_1_21_)
        conv82_1_21_ = self.conv82_1_21_(conv82_1_21_padding)
        batchnorm82_1_21_ = self.batchnorm82_1_21_(conv82_1_21_)
        relu82_1_21_ = self.relu82_1_21_(batchnorm82_1_21_)
        conv83_1_21_ = self.conv83_1_21_(relu82_1_21_)
        batchnorm83_1_21_ = self.batchnorm83_1_21_(conv83_1_21_)
        conv81_1_22_ = self.conv81_1_22_(relu79_)
        batchnorm81_1_22_ = self.batchnorm81_1_22_(conv81_1_22_)
        relu81_1_22_ = self.relu81_1_22_(batchnorm81_1_22_)
        conv82_1_22_padding = self.conv82_1_22_padding(relu81_1_22_)
        conv82_1_22_ = self.conv82_1_22_(conv82_1_22_padding)
        batchnorm82_1_22_ = self.batchnorm82_1_22_(conv82_1_22_)
        relu82_1_22_ = self.relu82_1_22_(batchnorm82_1_22_)
        conv83_1_22_ = self.conv83_1_22_(relu82_1_22_)
        batchnorm83_1_22_ = self.batchnorm83_1_22_(conv83_1_22_)
        conv81_1_23_ = self.conv81_1_23_(relu79_)
        batchnorm81_1_23_ = self.batchnorm81_1_23_(conv81_1_23_)
        relu81_1_23_ = self.relu81_1_23_(batchnorm81_1_23_)
        conv82_1_23_padding = self.conv82_1_23_padding(relu81_1_23_)
        conv82_1_23_ = self.conv82_1_23_(conv82_1_23_padding)
        batchnorm82_1_23_ = self.batchnorm82_1_23_(conv82_1_23_)
        relu82_1_23_ = self.relu82_1_23_(batchnorm82_1_23_)
        conv83_1_23_ = self.conv83_1_23_(relu82_1_23_)
        batchnorm83_1_23_ = self.batchnorm83_1_23_(conv83_1_23_)
        conv81_1_24_ = self.conv81_1_24_(relu79_)
        batchnorm81_1_24_ = self.batchnorm81_1_24_(conv81_1_24_)
        relu81_1_24_ = self.relu81_1_24_(batchnorm81_1_24_)
        conv82_1_24_padding = self.conv82_1_24_padding(relu81_1_24_)
        conv82_1_24_ = self.conv82_1_24_(conv82_1_24_padding)
        batchnorm82_1_24_ = self.batchnorm82_1_24_(conv82_1_24_)
        relu82_1_24_ = self.relu82_1_24_(batchnorm82_1_24_)
        conv83_1_24_ = self.conv83_1_24_(relu82_1_24_)
        batchnorm83_1_24_ = self.batchnorm83_1_24_(conv83_1_24_)
        conv81_1_25_ = self.conv81_1_25_(relu79_)
        batchnorm81_1_25_ = self.batchnorm81_1_25_(conv81_1_25_)
        relu81_1_25_ = self.relu81_1_25_(batchnorm81_1_25_)
        conv82_1_25_padding = self.conv82_1_25_padding(relu81_1_25_)
        conv82_1_25_ = self.conv82_1_25_(conv82_1_25_padding)
        batchnorm82_1_25_ = self.batchnorm82_1_25_(conv82_1_25_)
        relu82_1_25_ = self.relu82_1_25_(batchnorm82_1_25_)
        conv83_1_25_ = self.conv83_1_25_(relu82_1_25_)
        batchnorm83_1_25_ = self.batchnorm83_1_25_(conv83_1_25_)
        conv81_1_26_ = self.conv81_1_26_(relu79_)
        batchnorm81_1_26_ = self.batchnorm81_1_26_(conv81_1_26_)
        relu81_1_26_ = self.relu81_1_26_(batchnorm81_1_26_)
        conv82_1_26_padding = self.conv82_1_26_padding(relu81_1_26_)
        conv82_1_26_ = self.conv82_1_26_(conv82_1_26_padding)
        batchnorm82_1_26_ = self.batchnorm82_1_26_(conv82_1_26_)
        relu82_1_26_ = self.relu82_1_26_(batchnorm82_1_26_)
        conv83_1_26_ = self.conv83_1_26_(relu82_1_26_)
        batchnorm83_1_26_ = self.batchnorm83_1_26_(conv83_1_26_)
        conv81_1_27_ = self.conv81_1_27_(relu79_)
        batchnorm81_1_27_ = self.batchnorm81_1_27_(conv81_1_27_)
        relu81_1_27_ = self.relu81_1_27_(batchnorm81_1_27_)
        conv82_1_27_padding = self.conv82_1_27_padding(relu81_1_27_)
        conv82_1_27_ = self.conv82_1_27_(conv82_1_27_padding)
        batchnorm82_1_27_ = self.batchnorm82_1_27_(conv82_1_27_)
        relu82_1_27_ = self.relu82_1_27_(batchnorm82_1_27_)
        conv83_1_27_ = self.conv83_1_27_(relu82_1_27_)
        batchnorm83_1_27_ = self.batchnorm83_1_27_(conv83_1_27_)
        conv81_1_28_ = self.conv81_1_28_(relu79_)
        batchnorm81_1_28_ = self.batchnorm81_1_28_(conv81_1_28_)
        relu81_1_28_ = self.relu81_1_28_(batchnorm81_1_28_)
        conv82_1_28_padding = self.conv82_1_28_padding(relu81_1_28_)
        conv82_1_28_ = self.conv82_1_28_(conv82_1_28_padding)
        batchnorm82_1_28_ = self.batchnorm82_1_28_(conv82_1_28_)
        relu82_1_28_ = self.relu82_1_28_(batchnorm82_1_28_)
        conv83_1_28_ = self.conv83_1_28_(relu82_1_28_)
        batchnorm83_1_28_ = self.batchnorm83_1_28_(conv83_1_28_)
        conv81_1_29_ = self.conv81_1_29_(relu79_)
        batchnorm81_1_29_ = self.batchnorm81_1_29_(conv81_1_29_)
        relu81_1_29_ = self.relu81_1_29_(batchnorm81_1_29_)
        conv82_1_29_padding = self.conv82_1_29_padding(relu81_1_29_)
        conv82_1_29_ = self.conv82_1_29_(conv82_1_29_padding)
        batchnorm82_1_29_ = self.batchnorm82_1_29_(conv82_1_29_)
        relu82_1_29_ = self.relu82_1_29_(batchnorm82_1_29_)
        conv83_1_29_ = self.conv83_1_29_(relu82_1_29_)
        batchnorm83_1_29_ = self.batchnorm83_1_29_(conv83_1_29_)
        conv81_1_30_ = self.conv81_1_30_(relu79_)
        batchnorm81_1_30_ = self.batchnorm81_1_30_(conv81_1_30_)
        relu81_1_30_ = self.relu81_1_30_(batchnorm81_1_30_)
        conv82_1_30_padding = self.conv82_1_30_padding(relu81_1_30_)
        conv82_1_30_ = self.conv82_1_30_(conv82_1_30_padding)
        batchnorm82_1_30_ = self.batchnorm82_1_30_(conv82_1_30_)
        relu82_1_30_ = self.relu82_1_30_(batchnorm82_1_30_)
        conv83_1_30_ = self.conv83_1_30_(relu82_1_30_)
        batchnorm83_1_30_ = self.batchnorm83_1_30_(conv83_1_30_)
        conv81_1_31_ = self.conv81_1_31_(relu79_)
        batchnorm81_1_31_ = self.batchnorm81_1_31_(conv81_1_31_)
        relu81_1_31_ = self.relu81_1_31_(batchnorm81_1_31_)
        conv82_1_31_padding = self.conv82_1_31_padding(relu81_1_31_)
        conv82_1_31_ = self.conv82_1_31_(conv82_1_31_padding)
        batchnorm82_1_31_ = self.batchnorm82_1_31_(conv82_1_31_)
        relu82_1_31_ = self.relu82_1_31_(batchnorm82_1_31_)
        conv83_1_31_ = self.conv83_1_31_(relu82_1_31_)
        batchnorm83_1_31_ = self.batchnorm83_1_31_(conv83_1_31_)
        conv81_1_32_ = self.conv81_1_32_(relu79_)
        batchnorm81_1_32_ = self.batchnorm81_1_32_(conv81_1_32_)
        relu81_1_32_ = self.relu81_1_32_(batchnorm81_1_32_)
        conv82_1_32_padding = self.conv82_1_32_padding(relu81_1_32_)
        conv82_1_32_ = self.conv82_1_32_(conv82_1_32_padding)
        batchnorm82_1_32_ = self.batchnorm82_1_32_(conv82_1_32_)
        relu82_1_32_ = self.relu82_1_32_(batchnorm82_1_32_)
        conv83_1_32_ = self.conv83_1_32_(relu82_1_32_)
        batchnorm83_1_32_ = self.batchnorm83_1_32_(conv83_1_32_)
        add84_1_ = batchnorm83_1_1_ + batchnorm83_1_2_ + batchnorm83_1_3_ + batchnorm83_1_4_ + batchnorm83_1_5_ + batchnorm83_1_6_ + batchnorm83_1_7_ + batchnorm83_1_8_ + batchnorm83_1_9_ + batchnorm83_1_10_ + batchnorm83_1_11_ + batchnorm83_1_12_ + batchnorm83_1_13_ + batchnorm83_1_14_ + batchnorm83_1_15_ + batchnorm83_1_16_ + batchnorm83_1_17_ + batchnorm83_1_18_ + batchnorm83_1_19_ + batchnorm83_1_20_ + batchnorm83_1_21_ + batchnorm83_1_22_ + batchnorm83_1_23_ + batchnorm83_1_24_ + batchnorm83_1_25_ + batchnorm83_1_26_ + batchnorm83_1_27_ + batchnorm83_1_28_ + batchnorm83_1_29_ + batchnorm83_1_30_ + batchnorm83_1_31_ + batchnorm83_1_32_
        conv80_2_ = self.conv80_2_(relu79_)
        batchnorm80_2_ = self.batchnorm80_2_(conv80_2_)
        add85_ = add84_1_ + batchnorm80_2_
        relu85_ = self.relu85_(add85_)
        conv87_1_1_ = self.conv87_1_1_(relu85_)
        batchnorm87_1_1_ = self.batchnorm87_1_1_(conv87_1_1_)
        relu87_1_1_ = self.relu87_1_1_(batchnorm87_1_1_)
        conv88_1_1_padding = self.conv88_1_1_padding(relu87_1_1_)
        conv88_1_1_ = self.conv88_1_1_(conv88_1_1_padding)
        batchnorm88_1_1_ = self.batchnorm88_1_1_(conv88_1_1_)
        relu88_1_1_ = self.relu88_1_1_(batchnorm88_1_1_)
        conv89_1_1_ = self.conv89_1_1_(relu88_1_1_)
        batchnorm89_1_1_ = self.batchnorm89_1_1_(conv89_1_1_)
        conv87_1_2_ = self.conv87_1_2_(relu85_)
        batchnorm87_1_2_ = self.batchnorm87_1_2_(conv87_1_2_)
        relu87_1_2_ = self.relu87_1_2_(batchnorm87_1_2_)
        conv88_1_2_padding = self.conv88_1_2_padding(relu87_1_2_)
        conv88_1_2_ = self.conv88_1_2_(conv88_1_2_padding)
        batchnorm88_1_2_ = self.batchnorm88_1_2_(conv88_1_2_)
        relu88_1_2_ = self.relu88_1_2_(batchnorm88_1_2_)
        conv89_1_2_ = self.conv89_1_2_(relu88_1_2_)
        batchnorm89_1_2_ = self.batchnorm89_1_2_(conv89_1_2_)
        conv87_1_3_ = self.conv87_1_3_(relu85_)
        batchnorm87_1_3_ = self.batchnorm87_1_3_(conv87_1_3_)
        relu87_1_3_ = self.relu87_1_3_(batchnorm87_1_3_)
        conv88_1_3_padding = self.conv88_1_3_padding(relu87_1_3_)
        conv88_1_3_ = self.conv88_1_3_(conv88_1_3_padding)
        batchnorm88_1_3_ = self.batchnorm88_1_3_(conv88_1_3_)
        relu88_1_3_ = self.relu88_1_3_(batchnorm88_1_3_)
        conv89_1_3_ = self.conv89_1_3_(relu88_1_3_)
        batchnorm89_1_3_ = self.batchnorm89_1_3_(conv89_1_3_)
        conv87_1_4_ = self.conv87_1_4_(relu85_)
        batchnorm87_1_4_ = self.batchnorm87_1_4_(conv87_1_4_)
        relu87_1_4_ = self.relu87_1_4_(batchnorm87_1_4_)
        conv88_1_4_padding = self.conv88_1_4_padding(relu87_1_4_)
        conv88_1_4_ = self.conv88_1_4_(conv88_1_4_padding)
        batchnorm88_1_4_ = self.batchnorm88_1_4_(conv88_1_4_)
        relu88_1_4_ = self.relu88_1_4_(batchnorm88_1_4_)
        conv89_1_4_ = self.conv89_1_4_(relu88_1_4_)
        batchnorm89_1_4_ = self.batchnorm89_1_4_(conv89_1_4_)
        conv87_1_5_ = self.conv87_1_5_(relu85_)
        batchnorm87_1_5_ = self.batchnorm87_1_5_(conv87_1_5_)
        relu87_1_5_ = self.relu87_1_5_(batchnorm87_1_5_)
        conv88_1_5_padding = self.conv88_1_5_padding(relu87_1_5_)
        conv88_1_5_ = self.conv88_1_5_(conv88_1_5_padding)
        batchnorm88_1_5_ = self.batchnorm88_1_5_(conv88_1_5_)
        relu88_1_5_ = self.relu88_1_5_(batchnorm88_1_5_)
        conv89_1_5_ = self.conv89_1_5_(relu88_1_5_)
        batchnorm89_1_5_ = self.batchnorm89_1_5_(conv89_1_5_)
        conv87_1_6_ = self.conv87_1_6_(relu85_)
        batchnorm87_1_6_ = self.batchnorm87_1_6_(conv87_1_6_)
        relu87_1_6_ = self.relu87_1_6_(batchnorm87_1_6_)
        conv88_1_6_padding = self.conv88_1_6_padding(relu87_1_6_)
        conv88_1_6_ = self.conv88_1_6_(conv88_1_6_padding)
        batchnorm88_1_6_ = self.batchnorm88_1_6_(conv88_1_6_)
        relu88_1_6_ = self.relu88_1_6_(batchnorm88_1_6_)
        conv89_1_6_ = self.conv89_1_6_(relu88_1_6_)
        batchnorm89_1_6_ = self.batchnorm89_1_6_(conv89_1_6_)
        conv87_1_7_ = self.conv87_1_7_(relu85_)
        batchnorm87_1_7_ = self.batchnorm87_1_7_(conv87_1_7_)
        relu87_1_7_ = self.relu87_1_7_(batchnorm87_1_7_)
        conv88_1_7_padding = self.conv88_1_7_padding(relu87_1_7_)
        conv88_1_7_ = self.conv88_1_7_(conv88_1_7_padding)
        batchnorm88_1_7_ = self.batchnorm88_1_7_(conv88_1_7_)
        relu88_1_7_ = self.relu88_1_7_(batchnorm88_1_7_)
        conv89_1_7_ = self.conv89_1_7_(relu88_1_7_)
        batchnorm89_1_7_ = self.batchnorm89_1_7_(conv89_1_7_)
        conv87_1_8_ = self.conv87_1_8_(relu85_)
        batchnorm87_1_8_ = self.batchnorm87_1_8_(conv87_1_8_)
        relu87_1_8_ = self.relu87_1_8_(batchnorm87_1_8_)
        conv88_1_8_padding = self.conv88_1_8_padding(relu87_1_8_)
        conv88_1_8_ = self.conv88_1_8_(conv88_1_8_padding)
        batchnorm88_1_8_ = self.batchnorm88_1_8_(conv88_1_8_)
        relu88_1_8_ = self.relu88_1_8_(batchnorm88_1_8_)
        conv89_1_8_ = self.conv89_1_8_(relu88_1_8_)
        batchnorm89_1_8_ = self.batchnorm89_1_8_(conv89_1_8_)
        conv87_1_9_ = self.conv87_1_9_(relu85_)
        batchnorm87_1_9_ = self.batchnorm87_1_9_(conv87_1_9_)
        relu87_1_9_ = self.relu87_1_9_(batchnorm87_1_9_)
        conv88_1_9_padding = self.conv88_1_9_padding(relu87_1_9_)
        conv88_1_9_ = self.conv88_1_9_(conv88_1_9_padding)
        batchnorm88_1_9_ = self.batchnorm88_1_9_(conv88_1_9_)
        relu88_1_9_ = self.relu88_1_9_(batchnorm88_1_9_)
        conv89_1_9_ = self.conv89_1_9_(relu88_1_9_)
        batchnorm89_1_9_ = self.batchnorm89_1_9_(conv89_1_9_)
        conv87_1_10_ = self.conv87_1_10_(relu85_)
        batchnorm87_1_10_ = self.batchnorm87_1_10_(conv87_1_10_)
        relu87_1_10_ = self.relu87_1_10_(batchnorm87_1_10_)
        conv88_1_10_padding = self.conv88_1_10_padding(relu87_1_10_)
        conv88_1_10_ = self.conv88_1_10_(conv88_1_10_padding)
        batchnorm88_1_10_ = self.batchnorm88_1_10_(conv88_1_10_)
        relu88_1_10_ = self.relu88_1_10_(batchnorm88_1_10_)
        conv89_1_10_ = self.conv89_1_10_(relu88_1_10_)
        batchnorm89_1_10_ = self.batchnorm89_1_10_(conv89_1_10_)
        conv87_1_11_ = self.conv87_1_11_(relu85_)
        batchnorm87_1_11_ = self.batchnorm87_1_11_(conv87_1_11_)
        relu87_1_11_ = self.relu87_1_11_(batchnorm87_1_11_)
        conv88_1_11_padding = self.conv88_1_11_padding(relu87_1_11_)
        conv88_1_11_ = self.conv88_1_11_(conv88_1_11_padding)
        batchnorm88_1_11_ = self.batchnorm88_1_11_(conv88_1_11_)
        relu88_1_11_ = self.relu88_1_11_(batchnorm88_1_11_)
        conv89_1_11_ = self.conv89_1_11_(relu88_1_11_)
        batchnorm89_1_11_ = self.batchnorm89_1_11_(conv89_1_11_)
        conv87_1_12_ = self.conv87_1_12_(relu85_)
        batchnorm87_1_12_ = self.batchnorm87_1_12_(conv87_1_12_)
        relu87_1_12_ = self.relu87_1_12_(batchnorm87_1_12_)
        conv88_1_12_padding = self.conv88_1_12_padding(relu87_1_12_)
        conv88_1_12_ = self.conv88_1_12_(conv88_1_12_padding)
        batchnorm88_1_12_ = self.batchnorm88_1_12_(conv88_1_12_)
        relu88_1_12_ = self.relu88_1_12_(batchnorm88_1_12_)
        conv89_1_12_ = self.conv89_1_12_(relu88_1_12_)
        batchnorm89_1_12_ = self.batchnorm89_1_12_(conv89_1_12_)
        conv87_1_13_ = self.conv87_1_13_(relu85_)
        batchnorm87_1_13_ = self.batchnorm87_1_13_(conv87_1_13_)
        relu87_1_13_ = self.relu87_1_13_(batchnorm87_1_13_)
        conv88_1_13_padding = self.conv88_1_13_padding(relu87_1_13_)
        conv88_1_13_ = self.conv88_1_13_(conv88_1_13_padding)
        batchnorm88_1_13_ = self.batchnorm88_1_13_(conv88_1_13_)
        relu88_1_13_ = self.relu88_1_13_(batchnorm88_1_13_)
        conv89_1_13_ = self.conv89_1_13_(relu88_1_13_)
        batchnorm89_1_13_ = self.batchnorm89_1_13_(conv89_1_13_)
        conv87_1_14_ = self.conv87_1_14_(relu85_)
        batchnorm87_1_14_ = self.batchnorm87_1_14_(conv87_1_14_)
        relu87_1_14_ = self.relu87_1_14_(batchnorm87_1_14_)
        conv88_1_14_padding = self.conv88_1_14_padding(relu87_1_14_)
        conv88_1_14_ = self.conv88_1_14_(conv88_1_14_padding)
        batchnorm88_1_14_ = self.batchnorm88_1_14_(conv88_1_14_)
        relu88_1_14_ = self.relu88_1_14_(batchnorm88_1_14_)
        conv89_1_14_ = self.conv89_1_14_(relu88_1_14_)
        batchnorm89_1_14_ = self.batchnorm89_1_14_(conv89_1_14_)
        conv87_1_15_ = self.conv87_1_15_(relu85_)
        batchnorm87_1_15_ = self.batchnorm87_1_15_(conv87_1_15_)
        relu87_1_15_ = self.relu87_1_15_(batchnorm87_1_15_)
        conv88_1_15_padding = self.conv88_1_15_padding(relu87_1_15_)
        conv88_1_15_ = self.conv88_1_15_(conv88_1_15_padding)
        batchnorm88_1_15_ = self.batchnorm88_1_15_(conv88_1_15_)
        relu88_1_15_ = self.relu88_1_15_(batchnorm88_1_15_)
        conv89_1_15_ = self.conv89_1_15_(relu88_1_15_)
        batchnorm89_1_15_ = self.batchnorm89_1_15_(conv89_1_15_)
        conv87_1_16_ = self.conv87_1_16_(relu85_)
        batchnorm87_1_16_ = self.batchnorm87_1_16_(conv87_1_16_)
        relu87_1_16_ = self.relu87_1_16_(batchnorm87_1_16_)
        conv88_1_16_padding = self.conv88_1_16_padding(relu87_1_16_)
        conv88_1_16_ = self.conv88_1_16_(conv88_1_16_padding)
        batchnorm88_1_16_ = self.batchnorm88_1_16_(conv88_1_16_)
        relu88_1_16_ = self.relu88_1_16_(batchnorm88_1_16_)
        conv89_1_16_ = self.conv89_1_16_(relu88_1_16_)
        batchnorm89_1_16_ = self.batchnorm89_1_16_(conv89_1_16_)
        conv87_1_17_ = self.conv87_1_17_(relu85_)
        batchnorm87_1_17_ = self.batchnorm87_1_17_(conv87_1_17_)
        relu87_1_17_ = self.relu87_1_17_(batchnorm87_1_17_)
        conv88_1_17_padding = self.conv88_1_17_padding(relu87_1_17_)
        conv88_1_17_ = self.conv88_1_17_(conv88_1_17_padding)
        batchnorm88_1_17_ = self.batchnorm88_1_17_(conv88_1_17_)
        relu88_1_17_ = self.relu88_1_17_(batchnorm88_1_17_)
        conv89_1_17_ = self.conv89_1_17_(relu88_1_17_)
        batchnorm89_1_17_ = self.batchnorm89_1_17_(conv89_1_17_)
        conv87_1_18_ = self.conv87_1_18_(relu85_)
        batchnorm87_1_18_ = self.batchnorm87_1_18_(conv87_1_18_)
        relu87_1_18_ = self.relu87_1_18_(batchnorm87_1_18_)
        conv88_1_18_padding = self.conv88_1_18_padding(relu87_1_18_)
        conv88_1_18_ = self.conv88_1_18_(conv88_1_18_padding)
        batchnorm88_1_18_ = self.batchnorm88_1_18_(conv88_1_18_)
        relu88_1_18_ = self.relu88_1_18_(batchnorm88_1_18_)
        conv89_1_18_ = self.conv89_1_18_(relu88_1_18_)
        batchnorm89_1_18_ = self.batchnorm89_1_18_(conv89_1_18_)
        conv87_1_19_ = self.conv87_1_19_(relu85_)
        batchnorm87_1_19_ = self.batchnorm87_1_19_(conv87_1_19_)
        relu87_1_19_ = self.relu87_1_19_(batchnorm87_1_19_)
        conv88_1_19_padding = self.conv88_1_19_padding(relu87_1_19_)
        conv88_1_19_ = self.conv88_1_19_(conv88_1_19_padding)
        batchnorm88_1_19_ = self.batchnorm88_1_19_(conv88_1_19_)
        relu88_1_19_ = self.relu88_1_19_(batchnorm88_1_19_)
        conv89_1_19_ = self.conv89_1_19_(relu88_1_19_)
        batchnorm89_1_19_ = self.batchnorm89_1_19_(conv89_1_19_)
        conv87_1_20_ = self.conv87_1_20_(relu85_)
        batchnorm87_1_20_ = self.batchnorm87_1_20_(conv87_1_20_)
        relu87_1_20_ = self.relu87_1_20_(batchnorm87_1_20_)
        conv88_1_20_padding = self.conv88_1_20_padding(relu87_1_20_)
        conv88_1_20_ = self.conv88_1_20_(conv88_1_20_padding)
        batchnorm88_1_20_ = self.batchnorm88_1_20_(conv88_1_20_)
        relu88_1_20_ = self.relu88_1_20_(batchnorm88_1_20_)
        conv89_1_20_ = self.conv89_1_20_(relu88_1_20_)
        batchnorm89_1_20_ = self.batchnorm89_1_20_(conv89_1_20_)
        conv87_1_21_ = self.conv87_1_21_(relu85_)
        batchnorm87_1_21_ = self.batchnorm87_1_21_(conv87_1_21_)
        relu87_1_21_ = self.relu87_1_21_(batchnorm87_1_21_)
        conv88_1_21_padding = self.conv88_1_21_padding(relu87_1_21_)
        conv88_1_21_ = self.conv88_1_21_(conv88_1_21_padding)
        batchnorm88_1_21_ = self.batchnorm88_1_21_(conv88_1_21_)
        relu88_1_21_ = self.relu88_1_21_(batchnorm88_1_21_)
        conv89_1_21_ = self.conv89_1_21_(relu88_1_21_)
        batchnorm89_1_21_ = self.batchnorm89_1_21_(conv89_1_21_)
        conv87_1_22_ = self.conv87_1_22_(relu85_)
        batchnorm87_1_22_ = self.batchnorm87_1_22_(conv87_1_22_)
        relu87_1_22_ = self.relu87_1_22_(batchnorm87_1_22_)
        conv88_1_22_padding = self.conv88_1_22_padding(relu87_1_22_)
        conv88_1_22_ = self.conv88_1_22_(conv88_1_22_padding)
        batchnorm88_1_22_ = self.batchnorm88_1_22_(conv88_1_22_)
        relu88_1_22_ = self.relu88_1_22_(batchnorm88_1_22_)
        conv89_1_22_ = self.conv89_1_22_(relu88_1_22_)
        batchnorm89_1_22_ = self.batchnorm89_1_22_(conv89_1_22_)
        conv87_1_23_ = self.conv87_1_23_(relu85_)
        batchnorm87_1_23_ = self.batchnorm87_1_23_(conv87_1_23_)
        relu87_1_23_ = self.relu87_1_23_(batchnorm87_1_23_)
        conv88_1_23_padding = self.conv88_1_23_padding(relu87_1_23_)
        conv88_1_23_ = self.conv88_1_23_(conv88_1_23_padding)
        batchnorm88_1_23_ = self.batchnorm88_1_23_(conv88_1_23_)
        relu88_1_23_ = self.relu88_1_23_(batchnorm88_1_23_)
        conv89_1_23_ = self.conv89_1_23_(relu88_1_23_)
        batchnorm89_1_23_ = self.batchnorm89_1_23_(conv89_1_23_)
        conv87_1_24_ = self.conv87_1_24_(relu85_)
        batchnorm87_1_24_ = self.batchnorm87_1_24_(conv87_1_24_)
        relu87_1_24_ = self.relu87_1_24_(batchnorm87_1_24_)
        conv88_1_24_padding = self.conv88_1_24_padding(relu87_1_24_)
        conv88_1_24_ = self.conv88_1_24_(conv88_1_24_padding)
        batchnorm88_1_24_ = self.batchnorm88_1_24_(conv88_1_24_)
        relu88_1_24_ = self.relu88_1_24_(batchnorm88_1_24_)
        conv89_1_24_ = self.conv89_1_24_(relu88_1_24_)
        batchnorm89_1_24_ = self.batchnorm89_1_24_(conv89_1_24_)
        conv87_1_25_ = self.conv87_1_25_(relu85_)
        batchnorm87_1_25_ = self.batchnorm87_1_25_(conv87_1_25_)
        relu87_1_25_ = self.relu87_1_25_(batchnorm87_1_25_)
        conv88_1_25_padding = self.conv88_1_25_padding(relu87_1_25_)
        conv88_1_25_ = self.conv88_1_25_(conv88_1_25_padding)
        batchnorm88_1_25_ = self.batchnorm88_1_25_(conv88_1_25_)
        relu88_1_25_ = self.relu88_1_25_(batchnorm88_1_25_)
        conv89_1_25_ = self.conv89_1_25_(relu88_1_25_)
        batchnorm89_1_25_ = self.batchnorm89_1_25_(conv89_1_25_)
        conv87_1_26_ = self.conv87_1_26_(relu85_)
        batchnorm87_1_26_ = self.batchnorm87_1_26_(conv87_1_26_)
        relu87_1_26_ = self.relu87_1_26_(batchnorm87_1_26_)
        conv88_1_26_padding = self.conv88_1_26_padding(relu87_1_26_)
        conv88_1_26_ = self.conv88_1_26_(conv88_1_26_padding)
        batchnorm88_1_26_ = self.batchnorm88_1_26_(conv88_1_26_)
        relu88_1_26_ = self.relu88_1_26_(batchnorm88_1_26_)
        conv89_1_26_ = self.conv89_1_26_(relu88_1_26_)
        batchnorm89_1_26_ = self.batchnorm89_1_26_(conv89_1_26_)
        conv87_1_27_ = self.conv87_1_27_(relu85_)
        batchnorm87_1_27_ = self.batchnorm87_1_27_(conv87_1_27_)
        relu87_1_27_ = self.relu87_1_27_(batchnorm87_1_27_)
        conv88_1_27_padding = self.conv88_1_27_padding(relu87_1_27_)
        conv88_1_27_ = self.conv88_1_27_(conv88_1_27_padding)
        batchnorm88_1_27_ = self.batchnorm88_1_27_(conv88_1_27_)
        relu88_1_27_ = self.relu88_1_27_(batchnorm88_1_27_)
        conv89_1_27_ = self.conv89_1_27_(relu88_1_27_)
        batchnorm89_1_27_ = self.batchnorm89_1_27_(conv89_1_27_)
        conv87_1_28_ = self.conv87_1_28_(relu85_)
        batchnorm87_1_28_ = self.batchnorm87_1_28_(conv87_1_28_)
        relu87_1_28_ = self.relu87_1_28_(batchnorm87_1_28_)
        conv88_1_28_padding = self.conv88_1_28_padding(relu87_1_28_)
        conv88_1_28_ = self.conv88_1_28_(conv88_1_28_padding)
        batchnorm88_1_28_ = self.batchnorm88_1_28_(conv88_1_28_)
        relu88_1_28_ = self.relu88_1_28_(batchnorm88_1_28_)
        conv89_1_28_ = self.conv89_1_28_(relu88_1_28_)
        batchnorm89_1_28_ = self.batchnorm89_1_28_(conv89_1_28_)
        conv87_1_29_ = self.conv87_1_29_(relu85_)
        batchnorm87_1_29_ = self.batchnorm87_1_29_(conv87_1_29_)
        relu87_1_29_ = self.relu87_1_29_(batchnorm87_1_29_)
        conv88_1_29_padding = self.conv88_1_29_padding(relu87_1_29_)
        conv88_1_29_ = self.conv88_1_29_(conv88_1_29_padding)
        batchnorm88_1_29_ = self.batchnorm88_1_29_(conv88_1_29_)
        relu88_1_29_ = self.relu88_1_29_(batchnorm88_1_29_)
        conv89_1_29_ = self.conv89_1_29_(relu88_1_29_)
        batchnorm89_1_29_ = self.batchnorm89_1_29_(conv89_1_29_)
        conv87_1_30_ = self.conv87_1_30_(relu85_)
        batchnorm87_1_30_ = self.batchnorm87_1_30_(conv87_1_30_)
        relu87_1_30_ = self.relu87_1_30_(batchnorm87_1_30_)
        conv88_1_30_padding = self.conv88_1_30_padding(relu87_1_30_)
        conv88_1_30_ = self.conv88_1_30_(conv88_1_30_padding)
        batchnorm88_1_30_ = self.batchnorm88_1_30_(conv88_1_30_)
        relu88_1_30_ = self.relu88_1_30_(batchnorm88_1_30_)
        conv89_1_30_ = self.conv89_1_30_(relu88_1_30_)
        batchnorm89_1_30_ = self.batchnorm89_1_30_(conv89_1_30_)
        conv87_1_31_ = self.conv87_1_31_(relu85_)
        batchnorm87_1_31_ = self.batchnorm87_1_31_(conv87_1_31_)
        relu87_1_31_ = self.relu87_1_31_(batchnorm87_1_31_)
        conv88_1_31_padding = self.conv88_1_31_padding(relu87_1_31_)
        conv88_1_31_ = self.conv88_1_31_(conv88_1_31_padding)
        batchnorm88_1_31_ = self.batchnorm88_1_31_(conv88_1_31_)
        relu88_1_31_ = self.relu88_1_31_(batchnorm88_1_31_)
        conv89_1_31_ = self.conv89_1_31_(relu88_1_31_)
        batchnorm89_1_31_ = self.batchnorm89_1_31_(conv89_1_31_)
        conv87_1_32_ = self.conv87_1_32_(relu85_)
        batchnorm87_1_32_ = self.batchnorm87_1_32_(conv87_1_32_)
        relu87_1_32_ = self.relu87_1_32_(batchnorm87_1_32_)
        conv88_1_32_padding = self.conv88_1_32_padding(relu87_1_32_)
        conv88_1_32_ = self.conv88_1_32_(conv88_1_32_padding)
        batchnorm88_1_32_ = self.batchnorm88_1_32_(conv88_1_32_)
        relu88_1_32_ = self.relu88_1_32_(batchnorm88_1_32_)
        conv89_1_32_ = self.conv89_1_32_(relu88_1_32_)
        batchnorm89_1_32_ = self.batchnorm89_1_32_(conv89_1_32_)
        add90_1_ = batchnorm89_1_1_ + batchnorm89_1_2_ + batchnorm89_1_3_ + batchnorm89_1_4_ + batchnorm89_1_5_ + batchnorm89_1_6_ + batchnorm89_1_7_ + batchnorm89_1_8_ + batchnorm89_1_9_ + batchnorm89_1_10_ + batchnorm89_1_11_ + batchnorm89_1_12_ + batchnorm89_1_13_ + batchnorm89_1_14_ + batchnorm89_1_15_ + batchnorm89_1_16_ + batchnorm89_1_17_ + batchnorm89_1_18_ + batchnorm89_1_19_ + batchnorm89_1_20_ + batchnorm89_1_21_ + batchnorm89_1_22_ + batchnorm89_1_23_ + batchnorm89_1_24_ + batchnorm89_1_25_ + batchnorm89_1_26_ + batchnorm89_1_27_ + batchnorm89_1_28_ + batchnorm89_1_29_ + batchnorm89_1_30_ + batchnorm89_1_31_ + batchnorm89_1_32_
        add91_ = add90_1_ + relu85_
        relu91_ = self.relu91_(add91_)
        conv93_1_1_ = self.conv93_1_1_(relu91_)
        batchnorm93_1_1_ = self.batchnorm93_1_1_(conv93_1_1_)
        relu93_1_1_ = self.relu93_1_1_(batchnorm93_1_1_)
        conv94_1_1_padding = self.conv94_1_1_padding(relu93_1_1_)
        conv94_1_1_ = self.conv94_1_1_(conv94_1_1_padding)
        batchnorm94_1_1_ = self.batchnorm94_1_1_(conv94_1_1_)
        relu94_1_1_ = self.relu94_1_1_(batchnorm94_1_1_)
        conv95_1_1_ = self.conv95_1_1_(relu94_1_1_)
        batchnorm95_1_1_ = self.batchnorm95_1_1_(conv95_1_1_)
        conv93_1_2_ = self.conv93_1_2_(relu91_)
        batchnorm93_1_2_ = self.batchnorm93_1_2_(conv93_1_2_)
        relu93_1_2_ = self.relu93_1_2_(batchnorm93_1_2_)
        conv94_1_2_padding = self.conv94_1_2_padding(relu93_1_2_)
        conv94_1_2_ = self.conv94_1_2_(conv94_1_2_padding)
        batchnorm94_1_2_ = self.batchnorm94_1_2_(conv94_1_2_)
        relu94_1_2_ = self.relu94_1_2_(batchnorm94_1_2_)
        conv95_1_2_ = self.conv95_1_2_(relu94_1_2_)
        batchnorm95_1_2_ = self.batchnorm95_1_2_(conv95_1_2_)
        conv93_1_3_ = self.conv93_1_3_(relu91_)
        batchnorm93_1_3_ = self.batchnorm93_1_3_(conv93_1_3_)
        relu93_1_3_ = self.relu93_1_3_(batchnorm93_1_3_)
        conv94_1_3_padding = self.conv94_1_3_padding(relu93_1_3_)
        conv94_1_3_ = self.conv94_1_3_(conv94_1_3_padding)
        batchnorm94_1_3_ = self.batchnorm94_1_3_(conv94_1_3_)
        relu94_1_3_ = self.relu94_1_3_(batchnorm94_1_3_)
        conv95_1_3_ = self.conv95_1_3_(relu94_1_3_)
        batchnorm95_1_3_ = self.batchnorm95_1_3_(conv95_1_3_)
        conv93_1_4_ = self.conv93_1_4_(relu91_)
        batchnorm93_1_4_ = self.batchnorm93_1_4_(conv93_1_4_)
        relu93_1_4_ = self.relu93_1_4_(batchnorm93_1_4_)
        conv94_1_4_padding = self.conv94_1_4_padding(relu93_1_4_)
        conv94_1_4_ = self.conv94_1_4_(conv94_1_4_padding)
        batchnorm94_1_4_ = self.batchnorm94_1_4_(conv94_1_4_)
        relu94_1_4_ = self.relu94_1_4_(batchnorm94_1_4_)
        conv95_1_4_ = self.conv95_1_4_(relu94_1_4_)
        batchnorm95_1_4_ = self.batchnorm95_1_4_(conv95_1_4_)
        conv93_1_5_ = self.conv93_1_5_(relu91_)
        batchnorm93_1_5_ = self.batchnorm93_1_5_(conv93_1_5_)
        relu93_1_5_ = self.relu93_1_5_(batchnorm93_1_5_)
        conv94_1_5_padding = self.conv94_1_5_padding(relu93_1_5_)
        conv94_1_5_ = self.conv94_1_5_(conv94_1_5_padding)
        batchnorm94_1_5_ = self.batchnorm94_1_5_(conv94_1_5_)
        relu94_1_5_ = self.relu94_1_5_(batchnorm94_1_5_)
        conv95_1_5_ = self.conv95_1_5_(relu94_1_5_)
        batchnorm95_1_5_ = self.batchnorm95_1_5_(conv95_1_5_)
        conv93_1_6_ = self.conv93_1_6_(relu91_)
        batchnorm93_1_6_ = self.batchnorm93_1_6_(conv93_1_6_)
        relu93_1_6_ = self.relu93_1_6_(batchnorm93_1_6_)
        conv94_1_6_padding = self.conv94_1_6_padding(relu93_1_6_)
        conv94_1_6_ = self.conv94_1_6_(conv94_1_6_padding)
        batchnorm94_1_6_ = self.batchnorm94_1_6_(conv94_1_6_)
        relu94_1_6_ = self.relu94_1_6_(batchnorm94_1_6_)
        conv95_1_6_ = self.conv95_1_6_(relu94_1_6_)
        batchnorm95_1_6_ = self.batchnorm95_1_6_(conv95_1_6_)
        conv93_1_7_ = self.conv93_1_7_(relu91_)
        batchnorm93_1_7_ = self.batchnorm93_1_7_(conv93_1_7_)
        relu93_1_7_ = self.relu93_1_7_(batchnorm93_1_7_)
        conv94_1_7_padding = self.conv94_1_7_padding(relu93_1_7_)
        conv94_1_7_ = self.conv94_1_7_(conv94_1_7_padding)
        batchnorm94_1_7_ = self.batchnorm94_1_7_(conv94_1_7_)
        relu94_1_7_ = self.relu94_1_7_(batchnorm94_1_7_)
        conv95_1_7_ = self.conv95_1_7_(relu94_1_7_)
        batchnorm95_1_7_ = self.batchnorm95_1_7_(conv95_1_7_)
        conv93_1_8_ = self.conv93_1_8_(relu91_)
        batchnorm93_1_8_ = self.batchnorm93_1_8_(conv93_1_8_)
        relu93_1_8_ = self.relu93_1_8_(batchnorm93_1_8_)
        conv94_1_8_padding = self.conv94_1_8_padding(relu93_1_8_)
        conv94_1_8_ = self.conv94_1_8_(conv94_1_8_padding)
        batchnorm94_1_8_ = self.batchnorm94_1_8_(conv94_1_8_)
        relu94_1_8_ = self.relu94_1_8_(batchnorm94_1_8_)
        conv95_1_8_ = self.conv95_1_8_(relu94_1_8_)
        batchnorm95_1_8_ = self.batchnorm95_1_8_(conv95_1_8_)
        conv93_1_9_ = self.conv93_1_9_(relu91_)
        batchnorm93_1_9_ = self.batchnorm93_1_9_(conv93_1_9_)
        relu93_1_9_ = self.relu93_1_9_(batchnorm93_1_9_)
        conv94_1_9_padding = self.conv94_1_9_padding(relu93_1_9_)
        conv94_1_9_ = self.conv94_1_9_(conv94_1_9_padding)
        batchnorm94_1_9_ = self.batchnorm94_1_9_(conv94_1_9_)
        relu94_1_9_ = self.relu94_1_9_(batchnorm94_1_9_)
        conv95_1_9_ = self.conv95_1_9_(relu94_1_9_)
        batchnorm95_1_9_ = self.batchnorm95_1_9_(conv95_1_9_)
        conv93_1_10_ = self.conv93_1_10_(relu91_)
        batchnorm93_1_10_ = self.batchnorm93_1_10_(conv93_1_10_)
        relu93_1_10_ = self.relu93_1_10_(batchnorm93_1_10_)
        conv94_1_10_padding = self.conv94_1_10_padding(relu93_1_10_)
        conv94_1_10_ = self.conv94_1_10_(conv94_1_10_padding)
        batchnorm94_1_10_ = self.batchnorm94_1_10_(conv94_1_10_)
        relu94_1_10_ = self.relu94_1_10_(batchnorm94_1_10_)
        conv95_1_10_ = self.conv95_1_10_(relu94_1_10_)
        batchnorm95_1_10_ = self.batchnorm95_1_10_(conv95_1_10_)
        conv93_1_11_ = self.conv93_1_11_(relu91_)
        batchnorm93_1_11_ = self.batchnorm93_1_11_(conv93_1_11_)
        relu93_1_11_ = self.relu93_1_11_(batchnorm93_1_11_)
        conv94_1_11_padding = self.conv94_1_11_padding(relu93_1_11_)
        conv94_1_11_ = self.conv94_1_11_(conv94_1_11_padding)
        batchnorm94_1_11_ = self.batchnorm94_1_11_(conv94_1_11_)
        relu94_1_11_ = self.relu94_1_11_(batchnorm94_1_11_)
        conv95_1_11_ = self.conv95_1_11_(relu94_1_11_)
        batchnorm95_1_11_ = self.batchnorm95_1_11_(conv95_1_11_)
        conv93_1_12_ = self.conv93_1_12_(relu91_)
        batchnorm93_1_12_ = self.batchnorm93_1_12_(conv93_1_12_)
        relu93_1_12_ = self.relu93_1_12_(batchnorm93_1_12_)
        conv94_1_12_padding = self.conv94_1_12_padding(relu93_1_12_)
        conv94_1_12_ = self.conv94_1_12_(conv94_1_12_padding)
        batchnorm94_1_12_ = self.batchnorm94_1_12_(conv94_1_12_)
        relu94_1_12_ = self.relu94_1_12_(batchnorm94_1_12_)
        conv95_1_12_ = self.conv95_1_12_(relu94_1_12_)
        batchnorm95_1_12_ = self.batchnorm95_1_12_(conv95_1_12_)
        conv93_1_13_ = self.conv93_1_13_(relu91_)
        batchnorm93_1_13_ = self.batchnorm93_1_13_(conv93_1_13_)
        relu93_1_13_ = self.relu93_1_13_(batchnorm93_1_13_)
        conv94_1_13_padding = self.conv94_1_13_padding(relu93_1_13_)
        conv94_1_13_ = self.conv94_1_13_(conv94_1_13_padding)
        batchnorm94_1_13_ = self.batchnorm94_1_13_(conv94_1_13_)
        relu94_1_13_ = self.relu94_1_13_(batchnorm94_1_13_)
        conv95_1_13_ = self.conv95_1_13_(relu94_1_13_)
        batchnorm95_1_13_ = self.batchnorm95_1_13_(conv95_1_13_)
        conv93_1_14_ = self.conv93_1_14_(relu91_)
        batchnorm93_1_14_ = self.batchnorm93_1_14_(conv93_1_14_)
        relu93_1_14_ = self.relu93_1_14_(batchnorm93_1_14_)
        conv94_1_14_padding = self.conv94_1_14_padding(relu93_1_14_)
        conv94_1_14_ = self.conv94_1_14_(conv94_1_14_padding)
        batchnorm94_1_14_ = self.batchnorm94_1_14_(conv94_1_14_)
        relu94_1_14_ = self.relu94_1_14_(batchnorm94_1_14_)
        conv95_1_14_ = self.conv95_1_14_(relu94_1_14_)
        batchnorm95_1_14_ = self.batchnorm95_1_14_(conv95_1_14_)
        conv93_1_15_ = self.conv93_1_15_(relu91_)
        batchnorm93_1_15_ = self.batchnorm93_1_15_(conv93_1_15_)
        relu93_1_15_ = self.relu93_1_15_(batchnorm93_1_15_)
        conv94_1_15_padding = self.conv94_1_15_padding(relu93_1_15_)
        conv94_1_15_ = self.conv94_1_15_(conv94_1_15_padding)
        batchnorm94_1_15_ = self.batchnorm94_1_15_(conv94_1_15_)
        relu94_1_15_ = self.relu94_1_15_(batchnorm94_1_15_)
        conv95_1_15_ = self.conv95_1_15_(relu94_1_15_)
        batchnorm95_1_15_ = self.batchnorm95_1_15_(conv95_1_15_)
        conv93_1_16_ = self.conv93_1_16_(relu91_)
        batchnorm93_1_16_ = self.batchnorm93_1_16_(conv93_1_16_)
        relu93_1_16_ = self.relu93_1_16_(batchnorm93_1_16_)
        conv94_1_16_padding = self.conv94_1_16_padding(relu93_1_16_)
        conv94_1_16_ = self.conv94_1_16_(conv94_1_16_padding)
        batchnorm94_1_16_ = self.batchnorm94_1_16_(conv94_1_16_)
        relu94_1_16_ = self.relu94_1_16_(batchnorm94_1_16_)
        conv95_1_16_ = self.conv95_1_16_(relu94_1_16_)
        batchnorm95_1_16_ = self.batchnorm95_1_16_(conv95_1_16_)
        conv93_1_17_ = self.conv93_1_17_(relu91_)
        batchnorm93_1_17_ = self.batchnorm93_1_17_(conv93_1_17_)
        relu93_1_17_ = self.relu93_1_17_(batchnorm93_1_17_)
        conv94_1_17_padding = self.conv94_1_17_padding(relu93_1_17_)
        conv94_1_17_ = self.conv94_1_17_(conv94_1_17_padding)
        batchnorm94_1_17_ = self.batchnorm94_1_17_(conv94_1_17_)
        relu94_1_17_ = self.relu94_1_17_(batchnorm94_1_17_)
        conv95_1_17_ = self.conv95_1_17_(relu94_1_17_)
        batchnorm95_1_17_ = self.batchnorm95_1_17_(conv95_1_17_)
        conv93_1_18_ = self.conv93_1_18_(relu91_)
        batchnorm93_1_18_ = self.batchnorm93_1_18_(conv93_1_18_)
        relu93_1_18_ = self.relu93_1_18_(batchnorm93_1_18_)
        conv94_1_18_padding = self.conv94_1_18_padding(relu93_1_18_)
        conv94_1_18_ = self.conv94_1_18_(conv94_1_18_padding)
        batchnorm94_1_18_ = self.batchnorm94_1_18_(conv94_1_18_)
        relu94_1_18_ = self.relu94_1_18_(batchnorm94_1_18_)
        conv95_1_18_ = self.conv95_1_18_(relu94_1_18_)
        batchnorm95_1_18_ = self.batchnorm95_1_18_(conv95_1_18_)
        conv93_1_19_ = self.conv93_1_19_(relu91_)
        batchnorm93_1_19_ = self.batchnorm93_1_19_(conv93_1_19_)
        relu93_1_19_ = self.relu93_1_19_(batchnorm93_1_19_)
        conv94_1_19_padding = self.conv94_1_19_padding(relu93_1_19_)
        conv94_1_19_ = self.conv94_1_19_(conv94_1_19_padding)
        batchnorm94_1_19_ = self.batchnorm94_1_19_(conv94_1_19_)
        relu94_1_19_ = self.relu94_1_19_(batchnorm94_1_19_)
        conv95_1_19_ = self.conv95_1_19_(relu94_1_19_)
        batchnorm95_1_19_ = self.batchnorm95_1_19_(conv95_1_19_)
        conv93_1_20_ = self.conv93_1_20_(relu91_)
        batchnorm93_1_20_ = self.batchnorm93_1_20_(conv93_1_20_)
        relu93_1_20_ = self.relu93_1_20_(batchnorm93_1_20_)
        conv94_1_20_padding = self.conv94_1_20_padding(relu93_1_20_)
        conv94_1_20_ = self.conv94_1_20_(conv94_1_20_padding)
        batchnorm94_1_20_ = self.batchnorm94_1_20_(conv94_1_20_)
        relu94_1_20_ = self.relu94_1_20_(batchnorm94_1_20_)
        conv95_1_20_ = self.conv95_1_20_(relu94_1_20_)
        batchnorm95_1_20_ = self.batchnorm95_1_20_(conv95_1_20_)
        conv93_1_21_ = self.conv93_1_21_(relu91_)
        batchnorm93_1_21_ = self.batchnorm93_1_21_(conv93_1_21_)
        relu93_1_21_ = self.relu93_1_21_(batchnorm93_1_21_)
        conv94_1_21_padding = self.conv94_1_21_padding(relu93_1_21_)
        conv94_1_21_ = self.conv94_1_21_(conv94_1_21_padding)
        batchnorm94_1_21_ = self.batchnorm94_1_21_(conv94_1_21_)
        relu94_1_21_ = self.relu94_1_21_(batchnorm94_1_21_)
        conv95_1_21_ = self.conv95_1_21_(relu94_1_21_)
        batchnorm95_1_21_ = self.batchnorm95_1_21_(conv95_1_21_)
        conv93_1_22_ = self.conv93_1_22_(relu91_)
        batchnorm93_1_22_ = self.batchnorm93_1_22_(conv93_1_22_)
        relu93_1_22_ = self.relu93_1_22_(batchnorm93_1_22_)
        conv94_1_22_padding = self.conv94_1_22_padding(relu93_1_22_)
        conv94_1_22_ = self.conv94_1_22_(conv94_1_22_padding)
        batchnorm94_1_22_ = self.batchnorm94_1_22_(conv94_1_22_)
        relu94_1_22_ = self.relu94_1_22_(batchnorm94_1_22_)
        conv95_1_22_ = self.conv95_1_22_(relu94_1_22_)
        batchnorm95_1_22_ = self.batchnorm95_1_22_(conv95_1_22_)
        conv93_1_23_ = self.conv93_1_23_(relu91_)
        batchnorm93_1_23_ = self.batchnorm93_1_23_(conv93_1_23_)
        relu93_1_23_ = self.relu93_1_23_(batchnorm93_1_23_)
        conv94_1_23_padding = self.conv94_1_23_padding(relu93_1_23_)
        conv94_1_23_ = self.conv94_1_23_(conv94_1_23_padding)
        batchnorm94_1_23_ = self.batchnorm94_1_23_(conv94_1_23_)
        relu94_1_23_ = self.relu94_1_23_(batchnorm94_1_23_)
        conv95_1_23_ = self.conv95_1_23_(relu94_1_23_)
        batchnorm95_1_23_ = self.batchnorm95_1_23_(conv95_1_23_)
        conv93_1_24_ = self.conv93_1_24_(relu91_)
        batchnorm93_1_24_ = self.batchnorm93_1_24_(conv93_1_24_)
        relu93_1_24_ = self.relu93_1_24_(batchnorm93_1_24_)
        conv94_1_24_padding = self.conv94_1_24_padding(relu93_1_24_)
        conv94_1_24_ = self.conv94_1_24_(conv94_1_24_padding)
        batchnorm94_1_24_ = self.batchnorm94_1_24_(conv94_1_24_)
        relu94_1_24_ = self.relu94_1_24_(batchnorm94_1_24_)
        conv95_1_24_ = self.conv95_1_24_(relu94_1_24_)
        batchnorm95_1_24_ = self.batchnorm95_1_24_(conv95_1_24_)
        conv93_1_25_ = self.conv93_1_25_(relu91_)
        batchnorm93_1_25_ = self.batchnorm93_1_25_(conv93_1_25_)
        relu93_1_25_ = self.relu93_1_25_(batchnorm93_1_25_)
        conv94_1_25_padding = self.conv94_1_25_padding(relu93_1_25_)
        conv94_1_25_ = self.conv94_1_25_(conv94_1_25_padding)
        batchnorm94_1_25_ = self.batchnorm94_1_25_(conv94_1_25_)
        relu94_1_25_ = self.relu94_1_25_(batchnorm94_1_25_)
        conv95_1_25_ = self.conv95_1_25_(relu94_1_25_)
        batchnorm95_1_25_ = self.batchnorm95_1_25_(conv95_1_25_)
        conv93_1_26_ = self.conv93_1_26_(relu91_)
        batchnorm93_1_26_ = self.batchnorm93_1_26_(conv93_1_26_)
        relu93_1_26_ = self.relu93_1_26_(batchnorm93_1_26_)
        conv94_1_26_padding = self.conv94_1_26_padding(relu93_1_26_)
        conv94_1_26_ = self.conv94_1_26_(conv94_1_26_padding)
        batchnorm94_1_26_ = self.batchnorm94_1_26_(conv94_1_26_)
        relu94_1_26_ = self.relu94_1_26_(batchnorm94_1_26_)
        conv95_1_26_ = self.conv95_1_26_(relu94_1_26_)
        batchnorm95_1_26_ = self.batchnorm95_1_26_(conv95_1_26_)
        conv93_1_27_ = self.conv93_1_27_(relu91_)
        batchnorm93_1_27_ = self.batchnorm93_1_27_(conv93_1_27_)
        relu93_1_27_ = self.relu93_1_27_(batchnorm93_1_27_)
        conv94_1_27_padding = self.conv94_1_27_padding(relu93_1_27_)
        conv94_1_27_ = self.conv94_1_27_(conv94_1_27_padding)
        batchnorm94_1_27_ = self.batchnorm94_1_27_(conv94_1_27_)
        relu94_1_27_ = self.relu94_1_27_(batchnorm94_1_27_)
        conv95_1_27_ = self.conv95_1_27_(relu94_1_27_)
        batchnorm95_1_27_ = self.batchnorm95_1_27_(conv95_1_27_)
        conv93_1_28_ = self.conv93_1_28_(relu91_)
        batchnorm93_1_28_ = self.batchnorm93_1_28_(conv93_1_28_)
        relu93_1_28_ = self.relu93_1_28_(batchnorm93_1_28_)
        conv94_1_28_padding = self.conv94_1_28_padding(relu93_1_28_)
        conv94_1_28_ = self.conv94_1_28_(conv94_1_28_padding)
        batchnorm94_1_28_ = self.batchnorm94_1_28_(conv94_1_28_)
        relu94_1_28_ = self.relu94_1_28_(batchnorm94_1_28_)
        conv95_1_28_ = self.conv95_1_28_(relu94_1_28_)
        batchnorm95_1_28_ = self.batchnorm95_1_28_(conv95_1_28_)
        conv93_1_29_ = self.conv93_1_29_(relu91_)
        batchnorm93_1_29_ = self.batchnorm93_1_29_(conv93_1_29_)
        relu93_1_29_ = self.relu93_1_29_(batchnorm93_1_29_)
        conv94_1_29_padding = self.conv94_1_29_padding(relu93_1_29_)
        conv94_1_29_ = self.conv94_1_29_(conv94_1_29_padding)
        batchnorm94_1_29_ = self.batchnorm94_1_29_(conv94_1_29_)
        relu94_1_29_ = self.relu94_1_29_(batchnorm94_1_29_)
        conv95_1_29_ = self.conv95_1_29_(relu94_1_29_)
        batchnorm95_1_29_ = self.batchnorm95_1_29_(conv95_1_29_)
        conv93_1_30_ = self.conv93_1_30_(relu91_)
        batchnorm93_1_30_ = self.batchnorm93_1_30_(conv93_1_30_)
        relu93_1_30_ = self.relu93_1_30_(batchnorm93_1_30_)
        conv94_1_30_padding = self.conv94_1_30_padding(relu93_1_30_)
        conv94_1_30_ = self.conv94_1_30_(conv94_1_30_padding)
        batchnorm94_1_30_ = self.batchnorm94_1_30_(conv94_1_30_)
        relu94_1_30_ = self.relu94_1_30_(batchnorm94_1_30_)
        conv95_1_30_ = self.conv95_1_30_(relu94_1_30_)
        batchnorm95_1_30_ = self.batchnorm95_1_30_(conv95_1_30_)
        conv93_1_31_ = self.conv93_1_31_(relu91_)
        batchnorm93_1_31_ = self.batchnorm93_1_31_(conv93_1_31_)
        relu93_1_31_ = self.relu93_1_31_(batchnorm93_1_31_)
        conv94_1_31_padding = self.conv94_1_31_padding(relu93_1_31_)
        conv94_1_31_ = self.conv94_1_31_(conv94_1_31_padding)
        batchnorm94_1_31_ = self.batchnorm94_1_31_(conv94_1_31_)
        relu94_1_31_ = self.relu94_1_31_(batchnorm94_1_31_)
        conv95_1_31_ = self.conv95_1_31_(relu94_1_31_)
        batchnorm95_1_31_ = self.batchnorm95_1_31_(conv95_1_31_)
        conv93_1_32_ = self.conv93_1_32_(relu91_)
        batchnorm93_1_32_ = self.batchnorm93_1_32_(conv93_1_32_)
        relu93_1_32_ = self.relu93_1_32_(batchnorm93_1_32_)
        conv94_1_32_padding = self.conv94_1_32_padding(relu93_1_32_)
        conv94_1_32_ = self.conv94_1_32_(conv94_1_32_padding)
        batchnorm94_1_32_ = self.batchnorm94_1_32_(conv94_1_32_)
        relu94_1_32_ = self.relu94_1_32_(batchnorm94_1_32_)
        conv95_1_32_ = self.conv95_1_32_(relu94_1_32_)
        batchnorm95_1_32_ = self.batchnorm95_1_32_(conv95_1_32_)
        add96_1_ = batchnorm95_1_1_ + batchnorm95_1_2_ + batchnorm95_1_3_ + batchnorm95_1_4_ + batchnorm95_1_5_ + batchnorm95_1_6_ + batchnorm95_1_7_ + batchnorm95_1_8_ + batchnorm95_1_9_ + batchnorm95_1_10_ + batchnorm95_1_11_ + batchnorm95_1_12_ + batchnorm95_1_13_ + batchnorm95_1_14_ + batchnorm95_1_15_ + batchnorm95_1_16_ + batchnorm95_1_17_ + batchnorm95_1_18_ + batchnorm95_1_19_ + batchnorm95_1_20_ + batchnorm95_1_21_ + batchnorm95_1_22_ + batchnorm95_1_23_ + batchnorm95_1_24_ + batchnorm95_1_25_ + batchnorm95_1_26_ + batchnorm95_1_27_ + batchnorm95_1_28_ + batchnorm95_1_29_ + batchnorm95_1_30_ + batchnorm95_1_31_ + batchnorm95_1_32_
        add97_ = add96_1_ + relu91_
        relu97_ = self.relu97_(add97_)
        globalpooling97_ = self.globalpooling97_(relu97_)
        fc97_ = self.fc97_(globalpooling97_)
        softmax97_ = F.softmax(fc97_, axis=-1)
        predictions_ = F.identity(softmax97_)

        return [[predictions_]]
