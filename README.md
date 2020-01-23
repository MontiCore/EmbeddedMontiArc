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