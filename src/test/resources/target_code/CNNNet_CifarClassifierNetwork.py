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
                init=mx.init.Constant(data_mean.asnumpy().tolist()), differentiable=False)
            self.data_std = self.params.get('data_std', shape=data_mean.shape,
                init=mx.init.Constant(data_std.asnumpy().tolist()), differentiable=False)

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
    def __init__(self,batch_size, num_embeddings, embedding_dim, input_shape, total_feature_maps_size, **kwargs):
        super(VectorQuantize,self).__init__(**kwargs)
        self.embedding_dim = embedding_dim
        self.num_embeddings = num_embeddings
        self.input_shape = input_shape
        self.size = int(total_feature_maps_size / embedding_dim)

        # Codebook
        self.embeddings = self.params.get('embeddings', shape=(num_embeddings,embedding_dim), init=mx.init.Uniform())

    def hybrid_forward(self, F, x, *args, **params):
        # Flatten the inputs keeping and `embedding_dim` intact.
        flattened = F.reshape(x, shape=(-1, self.embedding_dim))

        # Get best representation
        a = F.broadcast_axis(F.sum(flattened ** 2, axis=1, keepdims=True), axis=1, size=self.num_embeddings)
        b = F.sum(F.broadcast_axis(F.expand_dims(self.embeddings.var() ** 2,axis=0),axis=0,size=self.size), axis=2)

        distances = a + b - 2 * F.dot(flattened, F.transpose(self.embeddings.var()))

        encoding_indices = F.argmin(distances, axis=1)
        encodings = F.one_hot(encoding_indices, self.num_embeddings)

        # Quantize and unflatten
        quantized = F.dot(encodings, self.embeddings.var()).reshape(self.input_shape)

        return quantized

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

            self.conv2_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv2_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv2_1_, output shape: {[8,32,32]}

            self.batchnorm2_1_ = gluon.nn.BatchNorm()
            # batchnorm2_1_, output shape: {[8,32,32]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.conv3_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv3_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv3_1_, output shape: {[8,32,32]}

            self.batchnorm3_1_ = gluon.nn.BatchNorm()
            # batchnorm3_1_, output shape: {[8,32,32]}

            self.conv2_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv2_2_, output shape: {[8,32,32]}

            self.batchnorm2_2_ = gluon.nn.BatchNorm()
            # batchnorm2_2_, output shape: {[8,32,32]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv5_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv5_1_, output shape: {[16,16,16]}

            self.batchnorm5_1_ = gluon.nn.BatchNorm()
            # batchnorm5_1_, output shape: {[16,16,16]}

            self.relu5_1_ = gluon.nn.Activation(activation='relu')
            self.conv6_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_1_, output shape: {[16,16,16]}

            self.batchnorm6_1_ = gluon.nn.BatchNorm()
            # batchnorm6_1_, output shape: {[16,16,16]}

            self.conv5_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv5_2_, output shape: {[16,16,16]}

            self.batchnorm5_2_ = gluon.nn.BatchNorm()
            # batchnorm5_2_, output shape: {[16,16,16]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.conv8_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv8_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv8_1_, output shape: {[16,16,16]}

            self.batchnorm8_1_ = gluon.nn.BatchNorm()
            # batchnorm8_1_, output shape: {[16,16,16]}

            self.relu8_1_ = gluon.nn.Activation(activation='relu')
            self.conv9_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv9_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv9_1_, output shape: {[16,16,16]}

            self.batchnorm9_1_ = gluon.nn.BatchNorm()
            # batchnorm9_1_, output shape: {[16,16,16]}

            self.relu10_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv11_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv11_1_, output shape: {[16,16,16]}

            self.batchnorm11_1_ = gluon.nn.BatchNorm()
            # batchnorm11_1_, output shape: {[16,16,16]}

            self.relu11_1_ = gluon.nn.Activation(activation='relu')
            self.conv12_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv12_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv12_1_, output shape: {[16,16,16]}

            self.batchnorm12_1_ = gluon.nn.BatchNorm()
            # batchnorm12_1_, output shape: {[16,16,16]}

            self.relu13_ = gluon.nn.Activation(activation='relu')
            self.conv14_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv14_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv14_1_, output shape: {[32,8,8]}

            self.batchnorm14_1_ = gluon.nn.BatchNorm()
            # batchnorm14_1_, output shape: {[32,8,8]}

            self.relu14_1_ = gluon.nn.Activation(activation='relu')
            self.conv15_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv15_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv15_1_, output shape: {[32,8,8]}

            self.batchnorm15_1_ = gluon.nn.BatchNorm()
            # batchnorm15_1_, output shape: {[32,8,8]}

            self.conv14_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv14_2_, output shape: {[32,8,8]}

            self.batchnorm14_2_ = gluon.nn.BatchNorm()
            # batchnorm14_2_, output shape: {[32,8,8]}

            self.relu16_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv17_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv17_1_, output shape: {[32,8,8]}

            self.batchnorm17_1_ = gluon.nn.BatchNorm()
            # batchnorm17_1_, output shape: {[32,8,8]}

            self.relu17_1_ = gluon.nn.Activation(activation='relu')
            self.conv18_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv18_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv18_1_, output shape: {[32,8,8]}

            self.batchnorm18_1_ = gluon.nn.BatchNorm()
            # batchnorm18_1_, output shape: {[32,8,8]}

            self.relu19_ = gluon.nn.Activation(activation='relu')
            self.conv20_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv20_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv20_1_, output shape: {[32,8,8]}

            self.batchnorm20_1_ = gluon.nn.BatchNorm()
            # batchnorm20_1_, output shape: {[32,8,8]}

            self.relu20_1_ = gluon.nn.Activation(activation='relu')
            self.conv21_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv21_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv21_1_, output shape: {[32,8,8]}

            self.batchnorm21_1_ = gluon.nn.BatchNorm()
            # batchnorm21_1_, output shape: {[32,8,8]}

            self.relu22_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv23_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv23_1_, output shape: {[64,4,4]}

            self.batchnorm23_1_ = gluon.nn.BatchNorm()
            # batchnorm23_1_, output shape: {[64,4,4]}

            self.relu23_1_ = gluon.nn.Activation(activation='relu')
            self.conv24_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv24_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv24_1_, output shape: {[64,4,4]}

            self.batchnorm24_1_ = gluon.nn.BatchNorm()
            # batchnorm24_1_, output shape: {[64,4,4]}

            self.conv23_2_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv23_2_, output shape: {[64,4,4]}

            self.batchnorm23_2_ = gluon.nn.BatchNorm()
            # batchnorm23_2_, output shape: {[64,4,4]}

            self.relu25_ = gluon.nn.Activation(activation='relu')
            self.conv26_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv26_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv26_1_, output shape: {[64,4,4]}

            self.batchnorm26_1_ = gluon.nn.BatchNorm()
            # batchnorm26_1_, output shape: {[64,4,4]}

            self.relu26_1_ = gluon.nn.Activation(activation='relu')
            self.conv27_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv27_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv27_1_, output shape: {[64,4,4]}

            self.batchnorm27_1_ = gluon.nn.BatchNorm()
            # batchnorm27_1_, output shape: {[64,4,4]}

            self.relu28_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv29_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv29_1_, output shape: {[64,4,4]}

            self.batchnorm29_1_ = gluon.nn.BatchNorm()
            # batchnorm29_1_, output shape: {[64,4,4]}

            self.relu29_1_ = gluon.nn.Activation(activation='relu')
            self.conv30_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv30_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv30_1_, output shape: {[64,4,4]}

            self.batchnorm30_1_ = gluon.nn.BatchNorm()
            # batchnorm30_1_, output shape: {[64,4,4]}

            self.relu31_ = gluon.nn.Activation(activation='relu')
            self.globalpooling31_ = gluon.nn.GlobalAvgPool2D()
            # globalpooling31_, output shape: {[64,1,1]}

            self.fc31_ = gluon.nn.Dense(units=128, use_bias=True, flatten=True)
            # fc31_, output shape: {[128,1,1]}

            self.dropout31_ = gluon.nn.Dropout(rate=0.5)
            self.fc32_ = gluon.nn.Dense(units=10, use_bias=True, flatten=True)
            # fc32_, output shape: {[10,1,1]}


            pass

    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)
        conv2_1_padding = self.conv2_1_padding(data_)
        conv2_1_ = self.conv2_1_(conv2_1_padding)
        batchnorm2_1_ = self.batchnorm2_1_(conv2_1_)
        relu2_1_ = self.relu2_1_(batchnorm2_1_)
        conv3_1_padding = self.conv3_1_padding(relu2_1_)
        conv3_1_ = self.conv3_1_(conv3_1_padding)
        batchnorm3_1_ = self.batchnorm3_1_(conv3_1_)
        conv2_2_ = self.conv2_2_(data_)
        batchnorm2_2_ = self.batchnorm2_2_(conv2_2_)
        add4_ = batchnorm3_1_ + batchnorm2_2_
        relu4_ = self.relu4_(add4_)
        conv5_1_padding = self.conv5_1_padding(relu4_)
        conv5_1_ = self.conv5_1_(conv5_1_padding)
        batchnorm5_1_ = self.batchnorm5_1_(conv5_1_)
        relu5_1_ = self.relu5_1_(batchnorm5_1_)
        conv6_1_padding = self.conv6_1_padding(relu5_1_)
        conv6_1_ = self.conv6_1_(conv6_1_padding)
        batchnorm6_1_ = self.batchnorm6_1_(conv6_1_)
        conv5_2_ = self.conv5_2_(relu4_)
        batchnorm5_2_ = self.batchnorm5_2_(conv5_2_)
        add7_ = batchnorm6_1_ + batchnorm5_2_
        relu7_ = self.relu7_(add7_)
        conv8_1_padding = self.conv8_1_padding(relu7_)
        conv8_1_ = self.conv8_1_(conv8_1_padding)
        batchnorm8_1_ = self.batchnorm8_1_(conv8_1_)
        relu8_1_ = self.relu8_1_(batchnorm8_1_)
        conv9_1_padding = self.conv9_1_padding(relu8_1_)
        conv9_1_ = self.conv9_1_(conv9_1_padding)
        batchnorm9_1_ = self.batchnorm9_1_(conv9_1_)
        add10_ = batchnorm9_1_ + relu7_
        relu10_ = self.relu10_(add10_)
        conv11_1_padding = self.conv11_1_padding(relu10_)
        conv11_1_ = self.conv11_1_(conv11_1_padding)
        batchnorm11_1_ = self.batchnorm11_1_(conv11_1_)
        relu11_1_ = self.relu11_1_(batchnorm11_1_)
        conv12_1_padding = self.conv12_1_padding(relu11_1_)
        conv12_1_ = self.conv12_1_(conv12_1_padding)
        batchnorm12_1_ = self.batchnorm12_1_(conv12_1_)
        add13_ = batchnorm12_1_ + relu10_
        relu13_ = self.relu13_(add13_)
        conv14_1_padding = self.conv14_1_padding(relu13_)
        conv14_1_ = self.conv14_1_(conv14_1_padding)
        batchnorm14_1_ = self.batchnorm14_1_(conv14_1_)
        relu14_1_ = self.relu14_1_(batchnorm14_1_)
        conv15_1_padding = self.conv15_1_padding(relu14_1_)
        conv15_1_ = self.conv15_1_(conv15_1_padding)
        batchnorm15_1_ = self.batchnorm15_1_(conv15_1_)
        conv14_2_ = self.conv14_2_(relu13_)
        batchnorm14_2_ = self.batchnorm14_2_(conv14_2_)
        add16_ = batchnorm15_1_ + batchnorm14_2_
        relu16_ = self.relu16_(add16_)
        conv17_1_padding = self.conv17_1_padding(relu16_)
        conv17_1_ = self.conv17_1_(conv17_1_padding)
        batchnorm17_1_ = self.batchnorm17_1_(conv17_1_)
        relu17_1_ = self.relu17_1_(batchnorm17_1_)
        conv18_1_padding = self.conv18_1_padding(relu17_1_)
        conv18_1_ = self.conv18_1_(conv18_1_padding)
        batchnorm18_1_ = self.batchnorm18_1_(conv18_1_)
        add19_ = batchnorm18_1_ + relu16_
        relu19_ = self.relu19_(add19_)
        conv20_1_padding = self.conv20_1_padding(relu19_)
        conv20_1_ = self.conv20_1_(conv20_1_padding)
        batchnorm20_1_ = self.batchnorm20_1_(conv20_1_)
        relu20_1_ = self.relu20_1_(batchnorm20_1_)
        conv21_1_padding = self.conv21_1_padding(relu20_1_)
        conv21_1_ = self.conv21_1_(conv21_1_padding)
        batchnorm21_1_ = self.batchnorm21_1_(conv21_1_)
        add22_ = batchnorm21_1_ + relu19_
        relu22_ = self.relu22_(add22_)
        conv23_1_padding = self.conv23_1_padding(relu22_)
        conv23_1_ = self.conv23_1_(conv23_1_padding)
        batchnorm23_1_ = self.batchnorm23_1_(conv23_1_)
        relu23_1_ = self.relu23_1_(batchnorm23_1_)
        conv24_1_padding = self.conv24_1_padding(relu23_1_)
        conv24_1_ = self.conv24_1_(conv24_1_padding)
        batchnorm24_1_ = self.batchnorm24_1_(conv24_1_)
        conv23_2_ = self.conv23_2_(relu22_)
        batchnorm23_2_ = self.batchnorm23_2_(conv23_2_)
        add25_ = batchnorm24_1_ + batchnorm23_2_
        relu25_ = self.relu25_(add25_)
        conv26_1_padding = self.conv26_1_padding(relu25_)
        conv26_1_ = self.conv26_1_(conv26_1_padding)
        batchnorm26_1_ = self.batchnorm26_1_(conv26_1_)
        relu26_1_ = self.relu26_1_(batchnorm26_1_)
        conv27_1_padding = self.conv27_1_padding(relu26_1_)
        conv27_1_ = self.conv27_1_(conv27_1_padding)
        batchnorm27_1_ = self.batchnorm27_1_(conv27_1_)
        add28_ = batchnorm27_1_ + relu25_
        relu28_ = self.relu28_(add28_)
        conv29_1_padding = self.conv29_1_padding(relu28_)
        conv29_1_ = self.conv29_1_(conv29_1_padding)
        batchnorm29_1_ = self.batchnorm29_1_(conv29_1_)
        relu29_1_ = self.relu29_1_(batchnorm29_1_)
        conv30_1_padding = self.conv30_1_padding(relu29_1_)
        conv30_1_ = self.conv30_1_(conv30_1_padding)
        batchnorm30_1_ = self.batchnorm30_1_(conv30_1_)
        add31_ = batchnorm30_1_ + relu28_
        relu31_ = self.relu31_(add31_)
        globalpooling31_ = self.globalpooling31_(relu31_)
        fc31_ = self.fc31_(globalpooling31_)
        dropout31_ = self.dropout31_(fc31_)
        fc32_ = self.fc32_(dropout31_)
        softmax32_ = F.softmax(fc32_, axis=-1)
        softmax_ = F.identity(softmax32_)

        return [[softmax_]]
