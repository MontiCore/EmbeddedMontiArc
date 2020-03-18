<#-- (c) https://github.com/MontiCore/monticore -->
import mxnet as mx
import numpy as np
import math
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

#Memory layer
class Memory(gluon.HybridBlock):
    def __init__(self, sub_key_size, query_size, act_query, k, num_heads, **kwargs):
        super(Memory, self).__init__(**kwargs)
        with self.name_scope():

            assert sub_key_size >= k

            if isinstance(query_size, int):
                sub_key_shape = (sub_key_size, query_size / 2)
                values_shape = (sub_key_size * sub_key_size, query_size)
            else:
                sub_key_shape = (sub_key_size, query_size[-1] / 2)
                values_shape = (sub_key_size * sub_key_size, query_size[-1])

            self.sub_keys1 = self.params.get("sub_keys1", shape=sub_key_shape)
            self.sub_keys2 = self.params.get("sub_keys2", shape=sub_key_shape)
            self.values = self.params.get("values", shape=values_shape)

            self.k = k

            self.num_heads = num_heads

            self.batch_norm = gluon.nn.BatchNorm()

            if isinstance(query_size, int):
                if isinstance(act_query, str):
                    self.query_net = []
                    for head_ind in range(num_heads):
                        if act_query == "linear":
                            self.query_net.append([gluon.nn.Dense(units=query_size)])
                        else:
                            self.query_net.append([gluon.nn.Dense(units=query_size, activation=act_query)])
                        self.register_child(self.query_net[head_ind][0])
                else:
                    self.query_net = []
                    for head_ind in range(num_heads):
                        head_net = []
                        for act in act_query:
                            if act == "linear":
                                head_net.append(gluon.nn.Dense(units=query_size))
                            else:
                                head_net.append(gluon.nn.Dense(units=query_size, activation=act))
                            self.register_child(head_net[-1])
                        self.query_net.append(head_net)
            else:
                self.query_net = []
                if isinstance(act_query, str):
                    for head_ind in range(num_heads):
                        head_net = []
                        for size in query_size:
                            if act_query == "linear":
                                head_net.append(gluon.nn.Dense(units=size))
                            else:
                                head_net.append(gluon.nn.Dense(units=size, activation=act_query))
                            self.register_child(head_net[-1])
                        self.query_net.append(head_net)
                else:
                    assert len(query_size) == len(act_query)
                    for head_ind in range(num_heads):
                        head_net = []
                        for size, act in zip(query_size, act_query):
                            if act == "linear":
                                head_net.append(gluon.nn.Dense(units=size))
                            else:
                                head_net.append(gluon.nn.Dense(units=size, activation=act))
                            self.register_child(head_net[-1])
                        self.query_net.append(head_net)
                        
    def hybrid_forward(self, F, x, sub_keys1, sub_keys2, values):
        x = self.batch_norm(x)

        for head_ind in range(self.num_heads):
            q = self.query_net[head_ind][0](x)
            if len(self.query_net[head_ind]) > 1:
                for layer in self.query_net[head_ind][1:]:
                    q = layer(q)

            q_split = F.split(q, num_outputs=2)
            q1_dot = F.dot(q_split[0], sub_keys1, transpose_b=True)
            q2_dot = F.dot(q_split[1], sub_keys2, transpose_b=True)
            i1 = F.topk(q1_dot, k=self.k, ret_typ="indices")
            i2 = F.topk(q2_dot, k=self.k, ret_typ="indices")

            # Calculate cross product for keys at indices I1 and I2
            k1 = F.take(sub_keys1, i1)
            k2 = F.take(sub_keys2, i2)

            k1 = F.tile(k1, (1, self.k, 1))
            k2 = F.repeat(k2, self.k, 1)
            c_cart = F.concat(k1, k2, dim=2)

            def loop_batch_dot(data, state):
                return F.dot(data[0], data[1], transpose_b=trans_b), state

            trans_b = True
            state_batch_dot = F.zeros(1)  # state is not used, but needed for function call
            (k_dot, _) = F.contrib.foreach(loop_batch_dot, [q, c_cart], init_states=state_batch_dot)

            i = F.topk(k_dot, k=self.k, ret_typ="both")
            w = F.softmax(i[0])

            vi = F.take(values, i[1])
            trans_b = False
            (aggr_value, _) = F.contrib.foreach(loop_batch_dot, [w, vi], init_states=state_batch_dot)

            if "m" in locals():
                m = F.broadcast_add(m, aggr_value)
            else:
                m = aggr_value

        return m


<#list tc.architecture.networkInstructions as networkInstruction>
class Net_${networkInstruction?index}(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}
            pass

    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}, attention_output_
<#else>
        return ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}
</#if>
</#list>

