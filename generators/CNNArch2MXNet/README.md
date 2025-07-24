<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2MXNet/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2MXNet/badges/master/coverage.svg)

## How to apply Data Cleaning
Missing, noisy and duplicate data can be classified as dirty data. Dirt data have an negative impact on the model performance. Therefore, one can apply data cleaning techniques like data removal to remove these kind of data entries. 
In order to apply data removal on our dataset, one can add a `cleaning` flag in the `Network.conf`:

```
cleaning: remove {
    missing:true
    duplicate:true
    noisy:true
}
```
The sub-parameters specifies if the corresponding dirty data type should be removed or not.

### Data Augmentation to counteract Data Imbalance

Data removal can cause data imbalance. To counter data imbalance, one can apply data up-sampling algorithms like data augmentation (e.g. image augmentation for images like the MNIST dataset). In order to apply image augmentation on our MNIST dataset, one can add a `data_augmentation` flag in the `Network.conf`:

```
data_imbalance: image_augmentation {
    rotation_angle:(-10,10,20) 
    shift:true
    scale_in:true
    scale_out:true
    check_bias:true
}
```
- `rotation_angle`: a list specifying the rotation degrees to be applied on an original image
- `shift`: speficies if a up, down, left and right shift should be applied
- `scale_in` and `scale_out`: specifies if the image should be scaled in and scaled out
- `check_bias`: specifies if the resulting up-sampled dataset should be checked for bias

