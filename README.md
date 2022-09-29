# CNNArch2PyTorch
The PyTorch backend

# Installation
Install the Stable PyTorch version using official instructions on [website](https://pytorch.org/get-started/locally/)

For installing the LibTorch for C++ predictions follow instructions on [URL](https://pytorch.org/cppdocs/installing.html)

For installing the DGL Library  [Follow the instructions from this link](https://www.dgl.ai/pages/start.html)

## Using Custom Layers

* Create a directory with following structure somewhere in your system :
<pre>
+-- custom_files
     +-- python
           +-- pytorch
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
