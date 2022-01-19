import dgl
from dgl.data import CoraGraphDataset
import mxnet as mx
import h5py
import numpy as np

if __name__ == "__main__":
    data = CoraGraphDataset()
    g = data[0]
    features = g.ndata['feat'].astype('float32')
    labels = g.ndata['label']

    g = dgl.remove_self_loop(g)
    g = dgl.add_self_loop(g)
    degs = g.in_degrees().astype('float32')
    norm = mx.nd.power(degs, -0.5)
    g.ndata['norm'] = mx.nd.expand_dims(norm, 1)
    features = features.asnumpy()
    labels = labels.asnumpy()
    features = np.broadcast_to(features, (1, features.shape[0], features.shape[1]))
    labels = np.broadcast_to(labels, (1, labels.shape[0]))
    print("Feature shape: ", features.shape, "Label shape: ", labels.shape)
    print("Feature type: ", features.dtype, "Label type: ", labels.dtype)

    dgl.save_graphs("training_data_dgl_cora/graph.h5", g)

    with h5py.File("training_data_dgl_cora/train.h5", "w") as ofile:
        in_feat_dset = ofile.create_dataset("features", data=features)
        out_pred_dset = ofile.create_dataset("predictions" + "_label", data=labels)

    with h5py.File("training_data_dgl_cora/test.h5", "w") as ofile:
        in_feat_dset = ofile.create_dataset("features", data=features)
        out_pred_dset = ofile.create_dataset("predictions" + "_label", data=labels)
    print("Done creating Datasets!")
