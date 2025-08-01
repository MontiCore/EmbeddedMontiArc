import logging
import mxnet as mx
import h5py
import os
from mxnet import gluon
import CNNCreator_de_rwth_montisim_agent_master_qnet as creator

def load_data(path):
    graph = None
    data = h5py.File(path, 'r')
    return data, graph


def predict(path_to_h5, start=None, end=None, batch_size=1, model_path="../../../../", context="cpu"):
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    data, graph = load_data(path_to_h5)
    if context == "gpu":
        mx_context = [mx.gpu()]
    else:
        if context != "cpu":
            logging.warning("%s is not a valid context argument. Switching to cpu..." % (context))
        mx_context = [mx.cpu()]

    net_creator = creator.CNNCreator_de_rwth_montisim_agent_master_qnet()
    net_creator._weights_dir_ = model_path + net_creator._model_dir_
    net_creator.construct(context=mx_context)
    net_creator._model_dir_ = net_creator._weights_dir_
    _ = net_creator.load(context=mx_context)

    data_dict = {}
    if 'state' in data:
        data_dict['state_'] = data['state'][start:end]
    output_names = ['action_label']
    if len(data_dict) == 0:
        label = {}
        for out in output_names:
            label[out] = data[out]
        data_dict = label
    iter = mx.io.NDArrayIter(data=data_dict, batch_size=batch_size)
    outputs = None
    batches = 0

    for _, batch in enumerate(iter):
        state_ = gluon.utils.split_and_load(batch.data[0], ctx_list=mx_context, even_split=False)
        net_ret = net_creator.networks[0](state_[0])
        action_ = [net_ret[0][0]]
        if outputs is None:
            outputs = action_[0]
        else:
            outputs = mx.nd.concat(outputs, action_[0], dim=0)
        batches += 1
    return outputs
