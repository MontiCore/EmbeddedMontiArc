import mxnet as mx
import numpy as np
import math
import os
import abc
import warnings
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
            mask = F.tile(mask, self.num_heads)
            mask = F.repeat(mask, self.dim_model)
            mask = F.reshape(mask, shape=(-1, self.dim_model))
        weights = F.softmax(score, mask, use_length=self.use_mask)

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

    def __init__(self, use_replay, replay_interval, replay_batch_size, replay_steps, replay_gradient_steps, num_heads, **kwargs):
        super(EpisodicReplayMemoryInterface, self).__init__(**kwargs)

        self.use_replay = use_replay
        self.replay_interval = replay_interval
        self.replay_batch_size = replay_batch_size
        self.replay_steps = replay_steps
        self.replay_gradient_steps = replay_gradient_steps
        self.num_heads = num_heads

    @abc.abstractmethod
    def store_samples(self, data, y, query_network, store_prob, mx_context):
        pass

    @abc.abstractmethod
    def sample_memory(self, batch_size, mx_context):
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
                 dist_measure,
                 k, 
                 num_heads,
                 values_dim,
                 **kwargs):
        super(LargeMemory, self).__init__(**kwargs)
        with self.name_scope():
            #Memory parameters
            self.dist_measure = dist_measure
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

        if self.dist_measure == "l2":
            q_split_resh = F.reshape(q_split[0], shape=(0,0,1,-1))
            sub_keys1_resh = F.reshape(sub_keys1, shape=(1,0,0,-1), reverse=True)
            q1_diff = F.broadcast_sub(q_split_resh, sub_keys1_resh)
            q1_dist = F.norm(q1_diff, axis=-1)
            q_split_resh = F.reshape(q_split[1], shape=(0,0,1,-1))
            sub_keys2_resh = F.reshape(sub_keys2, shape=(1,0,0,-1), reverse=True)
            q2_diff = F.broadcast_sub(q_split_resh, sub_keys2_resh)
            q2_dist = F.norm(q2_diff, axis=-1)
        else:
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

        # Calculate cross product for keys at indices I1 and I2

        # def head_take(data, state):
        #     return [F.take(data[0], data[2]), F.take(data[1], data[3])], state,
        #
        # i1 = F.transpose(i1, axes=(1,0,2))
        # i2 = F.transpose(i2, axes=(1, 0, 2))
        # st = F.zeros(1)
        # (k1, k2), _ = F.contrib.foreach(head_take, [sub_keys1, sub_keys2,i1,i2], st)
        # k1 = F.reshape(k1, shape=(-1, 0, 0), reverse=True)
        # k2 = F.reshape(k2, shape=(-1, 0, 0), reverse=True)
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
        if self.dist_measure == "l2":
            k_diff = F.broadcast_sub(q, c_cart)
            k_dist = F.norm(k_diff, axis=-1)
        else:
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
                 query_net_dir,
                 query_net_prefix,
                 query_net_num_inputs,
                 **kwargs):
        super(EpisodicMemory, self).__init__(use_replay, replay_interval, replay_batch_size, replay_steps, replay_gradient_steps, 1, **kwargs)
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


#Stream 0
class EpisodicSubNet_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(EpisodicSubNet_0, self).__init__(**kwargs)
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_data_ = ZScoreNormalization(data_mean=data_mean['data_'],
                                                                               data_std=data_std['data_'])
            else:
                self.input_normalization_data_ = NoNormalization()


    
            pass
    
    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)

        return [[data_]]

class EpisodicSubNet_1(gluon.HybridBlock):
    def __init__(self, mx_context=None, **kwargs):
        super(EpisodicSubNet_1, self).__init__(**kwargs)
        with self.name_scope():
            self.memory1_ = EpisodicMemory(replay_interval=78, replay_batch_size=100, replay_steps=1,
                                                replay_gradient_steps=1, store_prob=0.5,
                                                max_stored_samples=-1, memory_replacement_strategy="replace_oldest", use_replay=True,
                                                query_net_dir="pretrained/", 
                                                query_net_prefix="network_name-",
                                                query_net_num_inputs=1)
            self.fc1_ = gluon.nn.Dense(units=33, use_bias=True, flatten=True)
            # fc1_, output shape: {[33,1,1]}


    
            pass
    
    def hybrid_forward(self, F, *args):
        memory1_full_, ind_memory1_ = self.memory1_(*args)
        memory1_ = memory1_full_[0]
        fc1_ = self.fc1_(memory1_)

        retNames = [fc1_]
        ret = []
        for elem in retNames:
            if isinstance(elem, list) and len(elem) >= 2:
                for elem2 in elem: 
                    ret.append(elem2)
            else:
                ret.append(elem)
        return [ret, [memory1_full_, ind_memory1_]]

class EpisodicSubNet_2(gluon.HybridBlock):
    def __init__(self, mx_context=None, **kwargs):
        super(EpisodicSubNet_2, self).__init__(**kwargs)
        with self.name_scope():
            self.memory2_ = EpisodicMemory(replay_interval=78, replay_batch_size=100, replay_steps=1,
                                                replay_gradient_steps=1, store_prob=0.5,
                                                max_stored_samples=-1, memory_replacement_strategy="replace_oldest", use_replay=True,
                                                query_net_dir="pretrained/", 
                                                query_net_prefix="network_name-",
                                                query_net_num_inputs=1)
            self.fc2_ = gluon.nn.Dense(units=33, use_bias=True, flatten=True)
            # fc2_, output shape: {[33,1,1]}


    
            pass
    
    def hybrid_forward(self, F, *args):
        memory2_full_, ind_memory2_ = self.memory2_(*args)
        memory2_ = memory2_full_[0]
        fc2_ = self.fc2_(memory2_)

        retNames = [fc2_]
        ret = []
        for elem in retNames:
            if isinstance(elem, list) and len(elem) >= 2:
                for elem2 in elem: 
                    ret.append(elem2)
            else:
                ret.append(elem)
        return [ret, [memory2_full_, ind_memory2_]]

class EpisodicSubNet_3(gluon.HybridBlock):
    def __init__(self, mx_context=None, **kwargs):
        super(EpisodicSubNet_3, self).__init__(**kwargs)
        with self.name_scope():
            self.memory3_ = EpisodicMemory(replay_interval=78, replay_batch_size=100, replay_steps=1,
                                                replay_gradient_steps=1, store_prob=0.5,
                                                max_stored_samples=-1, memory_replacement_strategy="replace_oldest", use_replay=True,
                                                query_net_dir="pretrained/", 
                                                query_net_prefix="network_name-",
                                                query_net_num_inputs=1)
            self.fc3_ = gluon.nn.Dense(units=33, use_bias=True, flatten=True)
            # fc3_, output shape: {[33,1,1]}


    
            pass
    
    def hybrid_forward(self, F, *args):
        memory3_full_, ind_memory3_ = self.memory3_(*args)
        memory3_ = memory3_full_[0]
        fc3_ = self.fc3_(memory3_)
        softmax3_ = F.softmax(fc3_, axis=-1)
        softmax_ = F.identity(softmax3_)

        retNames = [softmax_]
        ret = []
        for elem in retNames:
            if isinstance(elem, list) and len(elem) >= 2:
                for elem2 in elem: 
                    ret.append(elem2)
            else:
                ret.append(elem)
        return [ret, [memory3_full_, ind_memory3_]]


class Net_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(Net_0, self).__init__(**kwargs)
        with self.name_scope():
            self.episodicsubnet0_ = EpisodicSubNet_0(data_mean, data_std, mx_context)

            self.episodic_sub_nets = []

            self.episodic_sub_nets.append(EpisodicSubNet_1(mx_context=mx_context))
            self.register_child(self.episodic_sub_nets[0])

            self.episodic_sub_nets.append(EpisodicSubNet_2(mx_context=mx_context))
            self.register_child(self.episodic_sub_nets[1])

            self.episodic_sub_nets.append(EpisodicSubNet_3(mx_context=mx_context))
            self.register_child(self.episodic_sub_nets[2])

            pass

    def hybrid_forward(self, F, data_):
        episodicsubnet0_ = self.episodicsubnet0_(data_)  
        episodicsubnet1_ = self.episodic_sub_nets[0](*episodicsubnet0_[0])
        episodicsubnet2_ = self.episodic_sub_nets[1](*episodicsubnet1_[0])
        episodicsubnet3_ = self.episodic_sub_nets[2](*episodicsubnet2_[0])
        return [episodicsubnet3_[0], [episodicsubnet1_[1], episodicsubnet2_[1], episodicsubnet3_[1], ]]

