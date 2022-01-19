import h5py
import numpy as np
import mxnet as mx
import argparse
import os
import sys
import logging
import dgl


def load_h5_files(data_path):
    if os.path.isfile(data_path):
        data_h5 = h5py.File(data_path, 'r')
        return data_h5
    else:
        print("Data loading failure. File '" + os.path.abspath(data_path) + "' does not exist.")
        sys.exit(1)


def preprocess(data, sample_size):
    adjacency = data['adjacency'][:]
    features = data['features'][:]
    features = np.swapaxes(features, 1, 2)
    feature_debug = features
    features = mx.nd.array(features)
    label = data['predictions_label'][:]
    sample_number = 0
    graph_list = []
    for sample in adjacency:
        src, dst = np.nonzero(sample)
        graph_sample = dgl.graph((src, dst))
        graph_list.append(dgl.add_self_loop(graph_sample))
        feature_sample = features[sample_number]
        feature_sample = feature_sample[0:graph_list[sample_number].num_nodes()]
        feature_sample = mx.nd.flatten(feature_sample)
        numpy_debug = feature_sample.asnumpy()
        graph_list[sample_number].ndata['features_'] = feature_sample
        sample_number += 1
    return graph_list, label


def print_data(data):
    for element in data:
        print(data[str(element)])


if __name__ == "__main__":
    train_data = load_h5_files("./training_data_ford/train_gnn.h5")
    print_data(train_data)
    train_graph, train_label = preprocess(train_data, 357)

    test_data = load_h5_files("./training_data_ford/test_gnn.h5")
    print_data(test_data)
    test_graph, test_label = preprocess(test_data, 40)

    # Save Train data
    dgl.save_graphs("training_data_ford/train_graph", train_graph)
    train_data.close()
    with h5py.File("training_data_ford/train.h5", "w") as ofile:
        train_label = ofile.create_dataset('predictions_label', data=train_label)

    # Save Test data
    dgl.save_graphs("./training_data_ford/test_graph", test_graph)
    test_data.close()
    with h5py.File("training_data_ford/test.h5", "w") as ofile:
        test_label = ofile.create_dataset('predictions_label', data=test_label)
