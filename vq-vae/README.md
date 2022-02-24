# VQ-VAEs
Vector-Quantized Variational Autoencoders are discrete variants of the VAEs.
Instead of a distribution of the latent codes the model learns a codebook using
the `VectorQuantized`-Layer.
We use a PixelCNN to generate new feature maps. This model learns a categorial probability distribution where each pixel receives a list of probabilities for each embedding vector of the codebook.
We sample for each pixel a codebook index and use it to construct the new feature maps which is then feed-forwarded to the decoder to generate new images.

PixelCNNs belong to the autoregressive generative models that are not yet modelable in EMADL as it is the case with GANs and VAEs. Therefore, the user needs to implement and train them themselves.


**This Model works only with (one) GPU.**
## How to Run
Generate code, train and build the EMADL model by executing:

```
bash build.sh
```


## Troubleshooting
### Training on RWTH HPC Cluster
If you get a stacktrace like this:
```
mxnet.base.MXNetError: MXNetError: Error in operator elemwise_add__mul_scalar_elemwise_sub:
Error in operator vectorquantize0__plus0:
[12:29:03] ../src/ndarray/./../operator/tensor/../elemwise_op_common.h:135:
Check failed: assign(&dattr, vec.at(i)):
Incompatible attr in node vectorquantize0__plus0 at 1-th input:
expected [4900,64], got [9800,64]
```
You must have used multiple GPUs. This happens because the VectorQuantized Layer is batch_size dependent however due to the multiple processing units the batch_size is split between all units.

You have to modify your CNNAutoencoderTrainer. Simply comment the line out where it gets all units:
```
num_pus = 1
if context == 'gpu':
    #num_pus = mx.context.num_gpus() <---- this one
    if num_pus >= 1:
        if num_pus == 1:
```

### Current Problem
The C++ runtime code has problems during inference of the shapes within the encoder model.
```
TestMNIST.cpp line  54:
    >> mnist_encoder encoder;
mnist_encoder.h line  12:
    >> CNNPredictor_mnist_encoder_0 _predictor_0_;
CNNPredictor_mnist_encoder.h line 145:
    >> network_symbols[0].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);
```
The processing of lines above will produce the following stacktrace:
```
terminate called after throwing an instance of 'dmlc::Error'
  what():  [08:24:52] /usr/include/mxnet-cpp/symbol.hpp:266: Check failed: MXSymbolInferShapeEx(GetHandle(), keys.size(), keys.data(), arg_ind_ptr.data(), arg_shape_data.data(), &in_shape_size, &in_shape_ndim, &in_shape_data, &out_shape_size, &out_shape_ndim, &out_shape_data, &aux_shape_size, &aux_shape_ndim, &aux_shape_data, &complete) == 0 (-1 vs. 0) : 
Stack trace:
  [bt] (0) ./build/cpp/TestMNIST(+0xaab9) [0x555da9349ab9]
  [bt] (1) ./build/cpp/TestMNIST(+0xd822) [0x555da934c822]
  [bt] (2) ./build/cpp/TestMNIST(+0x1e2bb) [0x555da935d2bb]
  [bt] (3) ./build/cpp/TestMNIST(+0x1d0e0) [0x555da935c0e0]
  [bt] (4) ./build/cpp/TestMNIST(+0x1eb01) [0x555da935db01]
  [bt] (5) ./build/cpp/TestMNIST(+0x74f0) [0x555da93464f0]
  [bt] (6) /lib/x86_64-linux-gnu/libc.so.6(__libc_start_main+0xe7) [0x7f950cccfbf7]
  [bt] (7) ./build/cpp/TestMNIST(+0x632a) [0x555da934532a]
```

This is also a problem regarding the batch_size dependent implementation.


