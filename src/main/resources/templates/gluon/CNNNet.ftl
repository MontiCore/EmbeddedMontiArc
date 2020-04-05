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
    def __init__(self, 
                 sub_key_size, 
                 query_size, 
                 query_act, 
                 k, 
                 num_heads,
                 value_shape,
                 **kwargs):
        super(Memory, self).__init__(**kwargs)
        with self.name_scope():

            #Parameter constraints
            assert sub_key_size >= k

            #Memory parameters
            self.k = k
            self.num_heads = num_heads

            #Batch norm sub-layer
            self.batch_norm = gluon.nn.BatchNorm()

            #Memory sub-layer
            sub_key_shape = (sub_key_size, query_size[-1] / 2)

            if value_shape == [-1]:
                value_shape = (sub_key_size*sub_key_size, query_size[-1])
            else:
                value_shape = (sub_key_size*sub_key_size,) + value_shape

            self.sub_keys1 = self.params.get("sub_keys1", shape=sub_key_shape, differentiable=True)
            self.sub_keys2 = self.params.get("sub_keys2", shape=sub_key_shape, differentiable=True)

            self.values = self.params.get("values", shape=value_shape, differentiable=True)

            self.query_net = []
            for head_ind in range(num_heads):
                head_net = []
                for size in query_size:
                    if query_act == "linear":
                        head_net.append(gluon.nn.Dense(units=size))
                    else:
                        head_net.append(gluon.nn.Dense(units=size, activation=query_act))
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

            if "mv" in locals():
                mv = F.broadcast_add(mv, aggr_value)
            else:
                mv = aggr_value

        return mv

#ReplayMemory layer
class ReplayMemory(gluon.HybridBlock):
    def __init__(self,
                 replay_interval,
                 replay_batch_size,
                 replay_steps,
                 replay_gradient_steps,
                 store_prob,
                 max_stored_samples,
                 **kwargs):
        super(ReplayMemory, self).__init__(**kwargs)
        with self.name_scope():

            #Replay parameters
            self.replay_interval = replay_interval
            self.replay_batch_size = replay_batch_size
            self.replay_steps = replay_steps
            self.replay_gradient_steps = replay_gradient_steps

            self.store_prob = int(store_prob*100)
            self.max_stored_samples = max_stored_samples

            #Memory
            self.value_memory = nd.array([])
            self.label_memory = [nd.array([])]

    def hybrid_forward(self, F, x):
            #propagate the input as the rest is only used for replay
            return x

    def store_samples(self, x, y, mx_context):
        x_values = x
        x_labels = y
        batch_size = x_values.shape[0]
        num_outputs = len(x_labels)


        zeros = nd.zeros(batch_size, ctx=mx_context)
        ones = nd.ones(batch_size, ctx=mx_context)
        rand = nd.random.randint(0, 100, batch_size, ctx=mx_context)
        ind = nd.where(rand < self.store_prob, ones, zeros, ctx=mx_context)

        to_store_values = nd.contrib.boolean_mask(x_values, ind)
        to_store_labels = [nd.contrib.boolean_mask(x_labels[i], ind) for i in range(num_outputs)]
        
        if len(to_store_values) != 0:
            if self.value_memory.shape[0] == 0:
                self.value_memory = to_store_values
                self.label_memory = to_store_labels
            elif self.max_stored_samples != -1 and self.value_memory.shape[0] >= self.max_stored_samples:
                num_to_store = to_store_values.shape[0]
                self.value_memory = nd.concat(self.value_memory[num_to_store:], to_store_values, dim=0)
                self.label_memory = [nd.concat(self.label_memory[i][num_to_store:], to_store_labels[i], dim=0) for i in range(num_outputs)]
            else:
                self.value_memory = nd.concat(self.value_memory, to_store_values, dim=0)
                self.label_memory = [nd.concat(self.label_memory[i], to_store_labels[i], dim=0) for i in range(num_outputs)]

    def sample_memory(self, batch_size, mx_context):

        num_stored_samples = self.value_memory.shape[0]
        if self.replay_batch_size == -1:
            sample_ind = nd.random.randint(0, num_stored_samples, (self.replay_steps, batch_size), ctx=mx_context)
        else:
            sample_ind = nd.random.normal(0, num_stored_samples, (self.replay_steps, self.replay_batch_size), ctx=mx_context)

        num_outputs = len(self.label_memory)

        sample_labels = [[self.label_memory[i][ind] for i in range(num_outputs)] for ind in sample_ind]
        sample_batches = [[self.value_memory[ind], sample_labels[i]] for i, ind in enumerate(sample_ind)]

        return sample_batches

<#list tc.architecture.networkInstructions as networkInstruction>

#Stream ${networkInstruction?index}
<#list networkInstruction.body.replaySubNetworks as elements>

class ReplaySubNet_${elements?index}(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
<#if elements?index == 0 >
        super(ReplaySubNet_${elements?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, elements?index, "ARCHITECTURE_DEFINITION")}
    
            pass
    
    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
${tc.include(networkInstruction.body, elements?index, "FORWARD_FUNCTION")}
        return [[${tc.getName(elements[elements?size-1])}]]
<#else>
        super(ReplaySubNet_${elements?index}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(networkInstruction.body, elements?index, "ARCHITECTURE_DEFINITION")}
    
            pass
    
    def hybrid_forward(self, F, x):
${tc.include(networkInstruction.body, elements?index, "FORWARD_FUNCTION")}
        return [[${tc.join(tc.getSubnetOutputNames(elements), ", ")}], [${tc.join(tc.getSubnetInputNames(elements), ", ")}]]
</#if>

</#list>

class Net_${networkInstruction?index}(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        with self.name_scope():
<#if networkInstruction.body.replaySubNetworks?has_content>
<#list networkInstruction.body.replaySubNetworks as elements>
<#if elements?index == 0>
            self.replaysubnet0_ = ReplaySubNet_${elements?index}(data_mean, data_std)

            self.replay_sub_nets = []

<#else>
            self.replay_sub_nets.append(ReplaySubNet_${elements?index}())
            self.register_child(self.replay_sub_nets[${elements?index - 1}])

</#if>
</#list>
<#else>
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}
</#if>    
        pass

    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
<#if networkInstruction.body.replaySubNetworks?has_content>
<#list networkInstruction.body.replaySubNetworks as elements>
<#if elements?index == 0>
        replaysubnet${elements?index}_ = self.replaysubnet${elements?index}_(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")})  
<#else>
        replaysubnet${elements?index}_ = self.replay_sub_nets[${elements?index-1}](replaysubnet${elements?index - 1}_[0][0])
</#if>
</#list>
        return [replaysubnet${networkInstruction.body.replaySubNetworks?size - 1}_[0], [<#list networkInstruction.body.replaySubNetworks as elements><#if elements?index != 0>replaysubnet${elements?index}_[1][0], </#if></#list>]]
<#else>
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
<#if tc.isAttentionNetwork() && networkInstruction.isUnroll() >
        return ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}, attention_output_
<#else>
        return ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ")}
</#if>
</#if>
</#list>

