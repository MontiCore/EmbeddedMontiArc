# MNIST-VAEs
This repository includes variants of VAEs to generate new handwritten digits using EMADL.

## Prerequisites
1. Ubuntu Linux 18.04 LTS (Probably works with different versions, too)
2. JDK 8
3. Deep Learning Framework **MXNet** (Recomended version >= 1.7.0)
4. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
5. OpenCV

## VAE Traningparameters

Parameter | Type                             | Default Value | Description       
---|----------------------------------|---------------|--------------------
`learning_method` | {`supervised`,`reinforcement`,`gan`,`vae`} | `supervised`  | !!! Must be set to `vae`. !!!                        
`encoder` | Component Path                   | -             | !!! Must set a reference to the encoder component. !!!
`batch_size` | N1 | `64`            | Size of mini batch.
`num_epoch` | N1 | `10` | Number of training iterations.
`normalize` | B | `false` | Normalization of the dataset.
`checkpoint_period` | N | `5` | Period of training iterations before saving network parameters. 
`load_checkpoint` | B | `false` | Loads the last saved networks and begins the training from the checkpoint.
`load_pretrained` | B | `false` | Loads the last saved networks and begins the training from the beginning.
`log_period` | N | `50` | Period of processed mini batches before creating a new log entry.
`reconstruction_loss` | {`mse`,`bce`} | `mse` | The reconstruction loss is either mean squarred error (mse) or in the case of binary data binary cross entropy (bce).
`print_images` | B | `false` | Saves images between a input image and reconstructed image that can be used to visually evaluate the training process.
`kl_loss_weight` | Q | `1.0` | Weight of the Kullbach-Leibler loss. Needed for beta-VAEs.
`optimizer` | {`sgd`,`adam`,`rmsprop`,`adagrad`,`nag`,`adadelta`} | `adam` | Optimization algorithm that will update the weights during training

Learning rate scheduling is also possible: Simply set the optimizer parameters `learning_rate_decay` and `step_size`. You can also set the `learning_rate` that will be used as initial value (default is 0.01).
The learning rate scheduler will return the next learning rate based on: 
```
learning_rate * pow(learning_rate_decay, floor(num_update/step_size))
```


