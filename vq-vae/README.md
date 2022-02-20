# VQ-VAEs
Vector-Quantized Variational Autoencoders are discrete variants of the VAEs. Instead of a distribution of the latent codes the model learns a codebook using the `VectorQuantized`-Layer. We use a PixelCNN to generate new feature maps for which we replace each pixel in each position with a codebook vector.

PixelCNNs belong to the autoregressive generative models that are not yet modelable in EMADL as it is the case with GANs and VAEs. Therefore, the user needs to implement and train them themselves.

## How to Run
Generate code, train and build the EMADL model by executing:

```
bash build.sh
```

Finally, run the generator as follows:
```
bash generate_digit.sh <code1> <code2>
```
where `<code1>` and `<code2>` are the values of type double for the 2D-latent code.



