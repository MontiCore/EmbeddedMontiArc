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
    # the parameters of the layer as keys and their data type as value

    def get_parameters(self):
        return {'alpha': int, 'ones': int}


    # A method to print a dictionary with
    # the parameters of the layer as keys and their data type as value

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
* When you use the script or directly use the EMADL2CPP generator to start generation code and training your model add the "-cfp" command line argument followed by the path to the custom_files folder
