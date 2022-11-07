<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2MXNet/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2MXNet/badges/master/coverage.svg)

## How to export inner network layers, e.g. an attention matrix
In order to visualize attention from an attention network, the data from this layer has to be returned from the network. Two steps are neccessary for that.
1.  The layer that should be exported has to be defined as a VariableSymbol. In order to do this, the keyword `layer` can be used in front of the respective layer. The definition of the layer has to be made before it is actually used in the network.
    For example, one could define an attention layer as `layer FullyConnected(units = 1, flatten=false) attention;` at the beginning of a network. By filling this layer with data, e.g. `
               input ->
               ... ->
               attention ->
               ...
`, the data in this layer will be saved until the end of the network iteration.
2.  In order to make the network return the saved data, the networks name must be added to the 'AllAttentionModels' class in this project. Furthermore, the layer must either be named `attention`, or the CNNNet.ftl template has to be adjusted to return differently named layers.

## Using Custom Layers

* Create a directory with following structure somewhere in your system :
<pre>
+-- custom_files
     +-- python
           +-- gluon
                +-- custom_layers
</pre>
* Add __init__.py file to the custom_layers folder and include a method in it to enable the usage of "from custom_layers import * " statement
* Place your custom layer python file inside the custom_layers folder (for the appropriate backend) and include the three functions in its class as shown in the example below
```
    # A method to return a dictionary with
    # the names of the parameters of the layer (string) as keys and their data type (data type object) as value

    def get_parameters(self):
        return {'alpha': int, 'ones': int}


    # A method to print a dictionary with
    # the names of the parameters of the layer (string) as keys and their data type (string) as value

    def print_parameters(self):
        print({'alpha': 'int', 'ones': 'int'})


    # A method which gets the output dimensions of the previous layer as array of 3-element tuples and computes
    # the output dimensions of this layer and print a 5-element tuple with the output dimensions and
    # min max value in the form (channel1, height1, width1, min1, max1), (channels2, ...)
    # input_dimension_array = [(channels1, height1, width1), (channels2, height2, width2), ...]
    
    def compute_output_types(self, alpha, ones, input_dimension_array):
        channels1 = input_dimension_array[0][0]
        height1 = input_dimension_array[0][1]
        width1 = input_dimension_array[0][2]
        min1 = 0
        max1 = 'oo'
        print([(channels1, height1, width1, min1, max1)])
```
* Use the custom layer inside the model with the same name as the file and the class inside is called
* When you use the script or directly use the EMADL2CPP generator to start generating code and training your model add the "-cfp" command line argument followed by the path to the custom_files folder

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


## How to use Hyperparameter Optimization

To carry out a Hyperparameter Optimization (HPO), one can set the `optimizer` flag as the nested parameter `hpo` like in the following:

```
optimizer: hpo {
    learning_rate_range: (0.00001, 1.0)
    weight_decay_range: (0.00001, 0.1) 
    momentum_range: (0.1, 0.9)
    optimizer_options: ("Adam", "SGD", "RMSProp", "AdaGrad", "NAG", "AdaDelta")
    with_cleaning:false
    ntrials:10
}
```

- `learning_rate_range`: minimum and maximum range of the learning rate hyperparameter
- `weight_decay_range`: minimum and maximum range of the weight decay hyperparameter
- `momentum_range`: minimum and maximum range of the momentum hyperparameter (only important for the NAG and SGD optimizer)
- `optimizer_options`: list of optimizers options for HPO
- `with_cleaning`: specifies if the cleaning parameters like `missing`, `noisy` and `duplicated` should be optimized as well
- `ntrials`: specifies how many trials a HPO should execute
