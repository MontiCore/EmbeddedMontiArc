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

#Large memory layer
class LargeMemory(gluon.HybridBlock):
    def __init__(self, numSubKeys, querrySize, k, **kwargs):
        super(LargeMemory, self).__init__(**kwargs)
        with self.name_scope():
            self.batchNorm = gluon.nn.BatchNorm()
            self.querryNet = gluon.nn.Dense(units=querrySize)  #TODO: (Maybe) maybe make an (nonlinear) multilayer optional
            
            subKeySize = (numSubKeys, querrySize/2)
            numValues = numSubKeys * numSubKeys
            self.subKeys1 = self.params.get("subKeys1", shape=subKeySize)
            self.subKeys2 = self.params.get("subKeys2", shape=subKeySize)
            self.values = self.params.get("values", shape = (numValues, querrySize))

            self.k = k

    def hybrid_forward(self, F, x, subKeys1, subKeys2, values):
        x = self.batchNorm(x)
        q = self.querryNet(x)
        qSplit = F.split(q, num_outputs=2) #TODO: (Maybe) make num outputs a parameter (number of sub keys)
        q1Dot = F.dot(qSplit[0], subKeys1, transpose_a=True)
        q2Dot = F.dot(qSplit[1], subKeys2, transpose_a=True)	    
        I1 = F.topk(q1Dot, k=self.k, ret_typ="indices")
        I2 = F.topk(q2Dot, k=self.k, ret_typ="indices")

        #Calculate cross product for keys at indices I1 and I2
        k1 = F.take(subKeys1, I1)
        k2 = F.take(subKeys2, I2)
        cCross = F.concat(k1[0], k2[0])
        for i in range(k):
            for j in range(k2):
                if i == j and i == 0:
                    break
                cCross = F.concat(cCross, F.Concat(k1[i], k2[j]))

        kDot = F.dot(q, cCross, transpose_a=True)
        I = F.topk(kDot, k=self.k, ret_typ="both")
        w = F.softmax(I[0])

        # TODO: Might be implemented more efficent with add_n
        indices = I[1][:][0] * 255 + I[1][:][1] #working?
        v = F.take(values, indices)
        temp = F.multiply(w[0], v[0])
        m = temp
        for i, vi in enumerate(v[1:]):
            temp = F.multiply(w[i], vi)
            m = F.add(m, temp)

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

