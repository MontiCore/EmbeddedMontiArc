import h5py
import numpy as np
import argparse
import os
import scipy.sparse as sp


def encode_onehot(labels):
    classes = set(labels)
    classes_dict = {c: np.identity(len(classes))[i, :] for i, c in enumerate(classes)}
    labels_onehot = np.array(list(map(classes_dict.get, labels)), dtype=np.int32)
    return labels_onehot


def normalize(matrix):
    rowsum = np.array(matrix.sum(1))
    r_inv = np.power(rowsum, -1).flatten()
    r_inv[np.isinf(r_inv)] = 0.
    r_mat_inv = sp.diags(r_inv)
    matrix = r_mat_inv.dot(matrix)
    return matrix


def load_data(path, dataset):
    print("Loading dataset")
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
    features = normalize(features)
    adj = normalize(adj + sp.eye(adj.shape[0]))
    return adj, features


if __name__ == "__main__":
    adj_matrix, feat_matrix = load_data("cora/", "cora")
