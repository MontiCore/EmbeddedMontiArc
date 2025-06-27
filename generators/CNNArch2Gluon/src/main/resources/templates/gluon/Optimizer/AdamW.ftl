<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import os
import numpy as np
import mxnet as mx
from mxnet import nd
from mxnet.ndarray.contrib import mp_adamw_update, adamw_update, multi_mp_adamw_update, multi_adamw_update
    
class AdamW(mx.optimizer.Optimizer):
    def __init__(self, learning_rate=0.001, beta1=0.9, beta2=0.999, epsilon=1e-8, **kwargs):
        super(AdamW, self).__init__(learning_rate=learning_rate, **kwargs)
        self.beta1 = beta1
        self.beta2 = beta2
        self.epsilon = epsilon
        self.aggregate_num = max(1, min(50, int(os.getenv('MXNET_OPTIMIZER_AGGREGATION_SIZE', '4'))))

    def create_state(self, _, weight):
        return (nd.zeros(weight.shape, weight.context, dtype=weight.dtype), #mean
                nd.zeros(weight.shape, weight.context, dtype=weight.dtype)) #variance

    def create_state_multi_precision(self, index, weight):
        weight_master_copy = None
        if self.multi_precision and weight.dtype == numpy.float16:
            weight_master_copy = weight.astype(numpy.float32)
            return (self.create_state(index, weight_master_copy), weight_master_copy)
        return self.create_state(index, weight)

    def update(self, index, weight, grad, state):
        self._update_impl(index, weight, grad, state, multi_precision=False)

    def update_multi_precision(self, index, weight, grad, state):
        use_multi_precision = self.multi_precision and weight[0].dtype == numpy.float16
        self._update_impl(index, weight, grad, state, multi_precision=use_multi_precision)

    def _update_impl(self, indices, weight, grad, state, multi_precision=False):
        """update function"""
        aggregate = self.aggregate_num > 1
        if not isinstance(indices, (tuple, list)):
            indices = [indices]
            weight = [weight]
            grad = [grad]
            state = [state]
        for w_i, g_i in zip(weight, grad):
            assert(isinstance(w_i, nd.NDArray))
            assert(isinstance(g_i, nd.NDArray))
            aggregate = (aggregate and
                         w_i.stype == 'default' and
                         g_i.stype == 'default')
        self._update_count(indices)
        lrs = self._get_lrs(indices)
        wds = self._get_wds(indices)

        # pylint: disable=access-member-before-definition
        if not isinstance(self.rescale_grad, nd.NDArray):
            self.rescale_grad = nd.full(shape=(1,), val=self.rescale_grad, ctx=weight[0].context)
        else:
            self.rescale_grad = self.rescale_grad.as_in_context(weight[0].context)

        kwargs = {'beta1': self.beta1, 'beta2': self.beta2, 'epsilon': self.epsilon,
                  'rescale_grad': self.rescale_grad}
        if self.clip_gradient:
            kwargs['clip_gradient'] = self.clip_gradient

        if aggregate:
            current_index = 0
            while current_index < len(indices):
                sidx = current_index
                eidx = min(current_index + self.aggregate_num, len(indices))
                if not multi_precision:
                    mean, var = list(zip(*state[sidx:eidx]))
                    multi_adamw_update(weight[sidx:eidx],
                                       grad[sidx:eidx],
                                       mean, var,
                                       out=weight[sidx:eidx],
                                       size=len(weight[sidx:eidx]),
                                       lrs=list(np.ones(len(weight[sidx:eidx]))),
                                       wds=wds[sidx:eidx],
                                       etas=lrs[sidx:eidx],
                                       **kwargs)
                else:
                    mean_var = list(zip(*state[sidx:eidx]))[0]
                    tmean_var = list(zip(*mean_var))
                    mean = tmean_var[0]
                    var = tmean_var[1]
                    multi_mp_adamw_update(weight[sidx:eidx],
                                          grad[sidx:eidx],
                                          mean, var,
                                          list(zip(*state[sidx:eidx]))[1],
                                          out=weight[sidx:eidx],
                                          size=len(weight[sidx:eidx]),
                                          lrs=list(np.ones(len(weight[sidx:eidx]))),
                                          wds=wds[sidx:eidx],
                                          etas=lrs[sidx:eidx],
                                          **kwargs)
                current_index += self.aggregate_num
        else:
            for w_i, g_i, s_i, lr, wd in zip(weight, grad, state, lrs, wds):
                if not multi_precision:
                    mean, var = s_i
                    adamw_update(w_i, g_i, mean, var, out=w_i, lr=1, wd=wd, eta=lr, **kwargs)
                else:
                    mean, var = s_i[0]
                    mp_adamw_update(w_i, g_i, mean, var, s_i[1], out=w_i, lr=1, wd=wd, eta=lr, **kwargs)    
