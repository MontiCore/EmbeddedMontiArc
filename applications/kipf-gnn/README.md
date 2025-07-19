# Graph Neural Networks
This repository contains applications using Graph Neural Networks in MontiAnna.

# Prerequisites
1. Ubuntu Linux 18.04 LTS (others untested)
2. Deep Learning Framework **MXNet** (tested with v.1.7.0)
3. Install **Deep Graph Library** v.0.6.1:
    ```
    pip3 install dgl==0.6.1
    ```
4. Set **MXNet** as **Deep Graph Library** backend:
    ```
    export DGLBACKEND=mxnet
    ```
    OR
    ```
    python3 -m dgl.backend.set_default_backend mxnet
    ```

# Parameters
We strongly advise to use the **DGL** when training Graph Neural Networks. To use it set `<useDgl>y</useDgl>` in the training configuration of the `pom.xml` of your project. Or when calling the generator directly use `-dgl y`. Currently, the Graph Neural Network Layers `GraphConv`, `GATConv`, `GraphAvgPool` and `GraphSumPool` are supported.

It is possible to train Graph Neural Networks without using the **DGL**, see `cora-custom-gnn` for an example. This requires the use of custom Graph Neural Network layers. 

# Schema
To use the Graph Neural Network training schema set `network_type:gnn` in the `.conf` file. Possible parameters:

`train_mask:(start_idx, end_idx)` and `test_mask(start_idx, end_idx)` use to mask nodes for single graph training.
`multi_graph:true` when using multiple graphs for training e.g., in graph level prediction tasks.