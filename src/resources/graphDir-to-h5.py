import h5py
import numpy as np
import argparse
import os
import scipy.sparse as sp
import sys


def encode_onehot(labels):
    classes = set(labels)
    classes_dict = {c: np.identity(len(classes))[i, :] for i, c in enumerate(classes)}
    labels_onehot = np.array(list(map(classes_dict.get, labels)), dtype=np.int32)
    return labels_onehot


def normalize_adj(matrix):
    rowsum = np.array(matrix.sum(1))
    r_inv = np.power(rowsum, -0.5).flatten()
    r_inv[np.isinf(r_inv)] = 0.
    r_mat_inv = sp.diags(r_inv)
    matrix = matrix.dot(r_mat_inv).transpose().dot(r_mat_inv)
    # coo_matrx = matrix.tocoo()
    return matrix


def normalize_feat(matrix):
    rowsum = np.array(matrix.sum(1))
    r_inv = np.power(rowsum, -1).flatten()
    r_inv[np.isinf(r_inv)] = 0.
    r_mat_inv = sp.diags(r_inv)
    matrix = r_mat_inv.dot(matrix)
    return matrix


def load_data(path, dataset):
    print("Loading dataset...")
    content = np.genfromtxt(path+dataset+".content", dtype=np.dtype(str))
    features = sp.csr_matrix(content[:, 1:-1], dtype=np.float32)
    labels = encode_onehot(content[:, -1])

    idx = np.array(content[:, 0], dtype=np.int32)
    idx_map = {j: i for i, j in enumerate(idx)}

    edges_unordered = np.genfromtxt(path+dataset+".cites", dtype=np.int32)
    edges = np.array(list(map(idx_map.get, edges_unordered.flatten())),
                     dtype=np.int32).reshape(edges_unordered.shape)
    adj = sp.coo_matrix((np.ones(edges.shape[0]), (edges[:, 0], edges[:, 1])),
                        shape=(labels.shape[0], labels.shape[0]),
                        dtype=np.float32)

    # build symmetric adjacency matrix
    adj = adj + adj.T.multiply(adj.T > adj) - adj.multiply(adj.T > adj)
    features = normalize_feat(features)
    adj = normalize_adj(adj + sp.eye(adj.shape[0]))
    return adj, features, labels


if __name__ == "__main__":
    in_feat = "features"
    in_adj = "adjacency"
    out_pred = "predictions"
    adj_matrix, feat_matrix, labels = load_data("cora/", "cora")
    adj_matrix = adj_matrix.todense()
    feat_matrix = feat_matrix.todense()
    nodes = feat_matrix.shape[0]
    features = feat_matrix.shape[1]
    classes = labels.shape[1]

    adj_matrix = np.broadcast_to(adj_matrix, (2, nodes, nodes))
    feat_matrix = np.broadcast_to(feat_matrix, (2, nodes, features))
    labels = np.broadcast_to(labels, (2, nodes, classes))

    print("Feature matrix shape: ", feat_matrix.shape)
    print("Adjacency matrix shape: ", adj_matrix.shape)
    print("Label matrix shape: ", labels.shape)

    with h5py.File("training_data/train.h5", "w") as ofile:
        in_feat_dset = ofile.create_dataset(in_feat, data=feat_matrix)
        in_adj_dset = ofile.create_dataset(in_adj, data=adj_matrix)
        out_pred_dset = ofile.create_dataset(out_pred + "_label", data=labels)
    with h5py.File("training_data/test.h5", "w") as ofile:
        in_feat_dset = ofile.create_dataset(in_feat, data=feat_matrix)
        in_adj_dset = ofile.create_dataset(in_adj, data=adj_matrix)
        out_pred_dset = ofile.create_dataset(out_pred + "_label", data=labels)



