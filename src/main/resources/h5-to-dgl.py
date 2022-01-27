import h5py
import numpy as np
import mxnet as mx
import argparse
import os
import sys
import logging
import dgl
import matplotlib.pyplot as plt


def load_h5_files(data_path):
    if os.path.isfile(data_path):
        data_h5 = h5py.File(data_path, 'r')
        return data_h5
    else:
        print("Data loading failure. File '" + os.path.abspath(data_path) + "' does not exist.")
        sys.exit(1)


def show_plot(data):
    x = range(0, len(data))
    y = data
    plt.title("Line graph")
    plt.xlabel("X axis")
    plt.ylabel("Y axis")
    plt.plot(x, y, color="green")
    plt.show()


def preprocess(data):
    adjacency = data['adjacency'][:]
    features = data['features'][:]
    graph_data = data['data'][:]
    features = np.swapaxes(features, 1, 2)
    features = mx.nd.array(features)
    label = data['predictions_label'][:]
    sample_number = 0
    graph_list = []
    for sample in adjacency:
        #if (sample_number % batch_size) == 0:
            #show_plot(label[sample_number])
        src, dst = np.nonzero(sample)
        graph_sample = dgl.graph((src, dst))
        graph_list.append(dgl.add_self_loop(graph_sample))
        feature_sample = features[sample_number]
        feature_sample = feature_sample[0:graph_list[sample_number].num_nodes()]
        feature_sample = mx.nd.flatten(feature_sample)
        graph_list[sample_number].ndata['features_'] = feature_sample
        sample_number += 1
    return graph_list, label, graph_data


def print_data(data):
    for element in data:
        print(data[str(element)])


if __name__ == "__main__":

    train_data = load_h5_files("./training_data_ford/train_gnn.h5")
    print_data(train_data)
    # 357 samples
    train_graph, train_label, train_graph_data = preprocess(train_data)

    test_data = load_h5_files("./training_data_ford/test_gnn.h5")
    print_data(test_data)
    # 40 samples
    test_graph, test_label, test_graph_data = preprocess(test_data)

    # Save Train data
    dgl.save_graphs("training_data_ford/train_graph", train_graph)
    train_data.close()
    with h5py.File("training_data_ford/train.h5", "w") as ofile:
        train_label = ofile.create_dataset('predictions_label', data=train_label)
        train_graph_data = ofile.create_dataset('graph_data', data=train_graph_data)

    # Save Test data
    dgl.save_graphs("./training_data_ford/test_graph", test_graph)
    test_data.close()
    with h5py.File("training_data_ford/test.h5", "w") as ofile:
        test_label = ofile.create_dataset('predictions_label', data=test_label)
        test_graph_data = ofile.create_dataset('graph_data', data=test_graph_data)

