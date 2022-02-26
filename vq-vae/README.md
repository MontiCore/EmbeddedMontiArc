# VQ-VAEs
Vector-Quantized Variational Autoencoders are discrete variants of the VAEs.
Instead of a distribution of the latent codes the model learns a codebook using
the `VectorQuantized`-Layer.

However, VQ-VAEs can not simply sample the codes from this codebook to generate new images.
They need to be appropriately composed. For that reason,
the VQ-VAEs need additionally a model that learns probability distributions of
feature map pixels from which we can take a sample and quantize them using the learned codebook.
We can use a autoregressive generative model like PixelCNNs.

These generative models are not yet modelable in EMADL.
Therefore, the user needs to implement and train them themselves.


## How to Run
Generate code, train and build the EMADL model by executing:

```
bash build.sh
```
## Note regarding pre-trained parameters
The parameters came from a model that was trained for 100 epochs. However, the minimum is at epoch 78. Due to the difference in reconstruction performance, the file model-0-newest-0000.params usually featuring parameters of epoch 99 were replaced with the model from epoch 78. Loss graph as well as reconstruction images during training can be found in the pre-trained folder.

Left image: Epoch 78; Right image: Epoch 99.

<img src="pre-trained/test_reconstruction_0_epoch78_batch_size500.png" alt="Epoch 78" width="350">
<img src="pre-trained/test_reconstruction_0_epoch99_batch_size500.png" alt="Epoch 99" width="350">


## Issues
The trained model does not work in C++, but the parameters can be loaded and used in Python. The reason why it doens not work will require further investigations. You can use the reconstruction tests in `src/test/test_pre-trained` as a reference point regarding this issue.