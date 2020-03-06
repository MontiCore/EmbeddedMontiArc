<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNArchLang/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNArchLang/badges/master/coverage.svg)

# CNNArch

**!!Attention: 
For all layers with padding: For the gluon backend currently only the default value is working ("same"). As there is and was the same
custom calculations done there is no behaviorial change to previous versions. See the following issue:
https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2x/issues/2
!!**

## Introduction
CNNArch is a descriptive language to model architectures of neural networks with support for convolutional and recurrent neural networks. 
It is being developed for use in the MontiCar language family, along with CNNTrain, which configures the training of the network, and EmbeddedMontiArcDL, which integrates CNNArch into EmbeddedMontiArc.
The inputs and outputs of a network are strongly typed and the validity of a network is checked at model creation.
In the following, we will explain the syntax and all features of CNNArch in combination with code examples to show how these can be used.

## Basic Structure
The syntax of this language has many similarities to python in the way how variables and methods are handled. 
Variables which occur only in form of parameters are seemingly untyped. 
However, the correctness of their values is checked at compile time.
The header of the architecture declares architecture parameters that can be used in all following expressions. 
In this way, different instances of the same architecture can be created.
The top part of the architecture consists of input, output and layer declarations.
The main part is the actual architecture definition that contains subnetworks in the form of a collection of architecture elements which are connected through the two operators "->" and "|". 
These subnetworks are delimited by semicolons. We allow multiple subnetworks so that they can be used for more complex architectures such as encoder-decoder architectures.
The architecture definition also contains the layer instance definitions that can be used to implement a layer in one subnetwork and access its internals such as its output or its state for RNNs in another subnetwork.
An architecture element can either be a layer, an input or output port, a constant node or a layer instance reference. 
The following is a complete example of the original version of Alexnet by A. Krizhevsky. 
There are more compact versions of the same architecture but we will get to that later. 
All predefined methods are listed at the end of this document.
```
architecture Alexnet_alt(img_height=224, img_width=224, img_channels=3, classes=10){
    def input Z(0:255)^{img_channels, img_height, img_width} image
    def output Q(0:1)^{classes} predictions

    image ->
    Convolution(kernel=(11,11), channels=96, stride=(4,4), padding="no_loss") ->
    Lrn(nsize=5, alpha=0.0001, beta=0.75) ->
    Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
    Relu() ->
    Split(n=2) ->
    (
        [0] ->
        Convolution(kernel=(5,5), channels=128) ->
        Lrn(nsize=5, alpha=0.0001, beta=0.75) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
        Relu()
    |
        [1] ->
        Convolution(kernel=(5,5), channels=128) ->
        Lrn(nsize=5, alpha=0.0001, beta=0.75) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
        Relu()
    ) ->
    Concatenate() ->
    Convolution(kernel=(3,3), channels=384) ->
    Relu() ->
    Split(n=2) ->
    (
        [0] ->
        Convolution(kernel=(3,3), channels=192) ->
        Relu() ->
        Convolution(kernel=(3,3), channels=128) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
        Relu()
    |
        [1] ->
        Convolution(kernel=(3,3), channels=192) ->
        Relu() ->
        Convolution(kernel=(3,3), channels=128) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
        Relu()
    ) ->
    Concatenate() ->
    FullyConnected(units=4096) ->
    Relu() ->
    Dropout() ->
    FullyConnected(units=4096) ->
    Relu() ->
    Dropout() ->
    FullyConnected(units=classes) ->
    Softmax() ->
    predictions;
}
```
*Note: The third convolutional and the first two fully connected layers are not divided into two streams like they are in the original Alexnet. 
This is done for the sake of simplicity. However, this change should not affect the actual computation.*

## Data Flow Operators
This language does not use symbols to denote a connections between layers like most deep learning frameworks but instead uses a approach which describes the data flow through the network. 
The first operator is the serial connection "->". The operator simply connects the output of the first element to the input of the second element. 
Despite being sequential in nature, CNNArch is still able to describe complex networks like ResNeXt through the use of the parallelization operator "|". 
This operator splits the network into parallel data streams. 
The serial connection operator has a higher precedence than the parallel connection operator. 
Therefore, it is necessary to use brackets around parallel groups of layers.
Each element in a parallel group has the same input stream as the whole group. 
The output of a parallelization block is a list of streams which can be merged into a single stream through use of the following layers: 
`Concatenate()`, `Add()` or `Get(index)`. 
Note: `Get(index=i)` can be abbreviated by `[i]`. 
The layer `Split(n)` in the example above creates multiple output streams from a single input stream by splitting the channels of the input data into *n* streams.


## Inputs and Outputs
An architecture in CNNArch can have multiple inputs and outputs. 
Multiple inputs (or outputs) of the same form can be combined to an array. 
Assuming `h` and `w` are architecture parameter, the following is a valid example:
```
def input Z(0:255)^{3, h, w} image[2]
def input Q(-oo:+oo)^{10} additionalData
def output Q(0:1)^{3} predictions
```
The first line defines the input *image* as an array of two color images with a resolution of `h` x `w`. 
The part `Z(0:255)`, which corresponds to the type definition in EmbeddedMontiArc, restricts the values to integers between 0 and 255. 
The following line `{3, h, w}` declares the shape of the input. 
The shape denotes the dimensionality in form  of depth (number of channels), height and width. 
Here, the height is initialized as `h`, the width as `w` and the number of channels is 3.  
The second line defines another input with one dimension of size 10 and arbitrary rational values. 
The last line defines an one-dimensional output of size 3 with rational values between 0 and 1 (probabilities of 3 classes).

If an input or output is an array, it can be used in the architecture in two different ways. 
Either a single element is accessed or the array is used as a whole. 
The line `image[0] ->` would access the first image of the array and `image ->` would directly result in 2 output streams. 
In fact, `image ->` is identical to `(image[0] | image[1]) ->`. 
Furthermore, assuming *out* is a output array of size 2, the line `-> out` would be identical to `-> ([0]->out[0] | [1]->out[1])`. 
Inputs and outputs can also be used in the middle of an architecture. 
In general, inputs create new streams and outputs consume existing streams.

## Layer Construction
It is possible to declare and construct new layers. The declaration of a layer is similar to methods in python. 
Each parameter can have a default value that makes it an optional argument. 
A new layer is constructed from other layers including other constructed layers. However, recursion is not allowed. 
The compiler will throw an error if recursion occurs. 
The following is a example of multiple layer declarations.
```
    def conv(kernel, channels, stride=1, act=true){
        Convolution(kernel=(filter,filter), channels=channels, stride=(stride,stride)) ->
        BatchNorm() ->
        Relu(?=act)
    }
    def resLayer(channels, stride=1, addSkipConv=false){
        (
            conv(kernel=3, channels=channels, stride=stride) ->
            conv(kernel=3, channels=channels, act=false)
        |
            conv(kernel=1, channels=channels, stride=stride, act=false, ?=addSkipConv)
        ) ->
        Add() ->
        Relu()
    }
```
The constructed layer `resLayer` in this example corresponds to a building block of a Residual Network. 
The `?` argument is a special argument which is explained in the next section.

## Introducing new Predefined Layers
In order to introduce a new predefined layer, the following steps are necessary:
*  A new class for the respective layer has to be introduced in the "predefined" folder of CNNArch.
*  The name of the layer and its arguments (if they are new) have to be added to the AllPredefinedLayers class in the "predefined" folder. The "create()" method for the new layer must be called in AllPredefinedLayers.
*  If the layer uses newly introduced arguments, a get() method for these arguments must be implemented in the CNNArch2X project, in the class "ArchitectureElementData".
*  Finally, the class must be implemented for the respective backend, e.g. by adding a corresponding .ftl file to the "elements" folder in CNNArch2Gluon, or similarly for other backends.

## Structural Arguments
Structural arguments are special arguments which can be set for each layer and which do not correspond to a layer parameter. 
The three structural arguments are "?", "->" and "|". The conditional argument "?" is a boolean. 
It does nothing if it is true and it removes the layer completely if it is false. 
This argument is only useful for layer construction. 
The other two structural arguments are non-negative integers which repeat the layer *x* number of times where *x* is equal to their value. 
The layer operator between each repetition has the same symbol as the argument.

Assuming `a` is a method without required arguments, 
then `a(-> = 3)->` is equal to `a()->a()->a()->`, 
`a(| = 3)->` is equal to `(a() | a() | a())->` and 
`a(-> = 3, | = 2)->` is equal to `(a()->a()->a() | a()->a()->a())->`. 

## Argument Sequences
Argument sequences can be used instead of regular arguments to declare that a layer should be repeated with the values of the given sequence. 
The operator between these so stacked layers is also given by the sequence. 
Other arguments that only have a single value are neutral to the repetition 
which means that the single value will be repeated an arbitrary number of times without having an influence on the number of repetitions.

The following are valid sequences: `[1->2->3->4]`, `[true | false]`, `{[1 | 3->2]`, `[ |2->3]` and `[1->..->4]`. 
All values in these examples could also be replaced by variable names or arithmetic or logical expressions. 
The last sequence is defined as a range and equal to the first one. A range in CNNArch is closed which means the start and end value are both in the sequence. 
Moreover, a range has always a step size of +1. Thus, the range `[0|..|-4]` would be empty. 
The data flow operators can be used both in the same argument sequence in which case a single parallelization block is created. 
A parallel group in this block can be empty, which is why `[ |2->3]` is a valid sequence. 
If a method contains multiple argument sequences, the language will try to combine them by expanding the smaller one and will throw an error at model creation if this fails.
Let `m` be a layer with parameters `a`, `b` and `c`, then the expression `m(a=[3->2],b=1)` is equal to `m(a=3,b=1)->m(a=2,b=1)`. 
Furthermore, the line `m(a=[5->3],b=[3|4|2],c=2)->` is equal to:
```
(
    m(a=5, b=3, c=2) ->
    m(a=3, b=3, c=2)
|
    m(a=5, b=4, c=2) ->
    m(a=3, b=4, c=2)
|
    m(a=5, b=2, c=2) ->
    m(a=3, b=2, c=2)
) ->
```
And `m(a=[|5|3->4], b=[|1|2], c=2)` is equal to: 
```
(

|
    m(a=5, b=1, c=2)
|
    m(a=3, b=2, c=2) ->
    m(a=4, b=2, c=2)
) ->
```
However, `m(a=[5->3], b=[2|4->6], c=2)->` and `m(a=[5->3], b=[2->4->6], c=2)->` would fail because it is not possible to expand *a* such that it is the same size as *b*.

## Expressions
This language supports the basic arithmetic operators "+", "-", "\*", "/", the logical operators "&&", "||", the comparison operators "==", "!=", "<", ">", "<=", ">=" 
and the constants `true` and `false`. 
At the moment, it is sometimes necessary to use parentheses around an expression to avoid a parsing error. 
For example, the line `someMethod(booleanArg = (1!=1))` does not parse without the parentheses around `1!=1`.

## Advanced Examples
This version of Alexnet, which uses method construction, argument sequences and special arguments, is identical to the one in the section Basic Structure.
```
architecture Alexnet_alt2(img_height=224, img_width=224, img_channels=3, classes=10){
    def input Z(0:255)^{img_channels, img_height, img_width} image
    def output Q(0:1)^{classes} predictions
    
    def conv(filter, channels, convStride=1, poolStride=1, hasLrn=false, convPadding="same"){
    	Convolution(kernel=(filter,filter), channels=channels, stride=(convStride,convStride), padding=convPadding) ->
        Lrn(nsize=5, alpha=0.0001, beta=0.75, ?=hasLrn) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(poolStride,poolStride), padding="no_loss", ?=(poolStride != 1)) ->
        Relu()
    }
    def split1(i){
        [i] ->
        conv(filter=5, channels=128, poolStride=2, hasLrn=true)
    }
    def split2(i){
        [i] ->
        conv(filter=3, channels=192) ->
        conv(filter=3, channels=128, poolStride=2)
    }
    def fc(){
        FullyConnected(units=4096) ->
        Relu() ->
        Dropout()
    }

    image ->
    conv(filter=11, channels=96, convStride=4, poolStride=2, hasLrn=true, convPadding="no_loss") ->
    Split(n=2) ->
    split1(i=[0|1]) ->
    Concatenate() ->
    conv(filter=3, channels=384) ->
    Split(n=2) ->
    split2(i=[0|1]) ->
    Concatenate() ->
    fc(-> = 2) ->
    FullyConnected(units=classes) ->
    Softmax() ->
    predictions;
}
```

The following architecture is the extremely deep ResNet-152.
```
architecture ResNet152(img_height=224, img_width=224, img_channels=3, classes=1000){
    def input Z(0:255)^{img_channels, img_height, img_width} data
    def output Q(0:1)^{classes} predictions

    def conv(kernel, channels, stride=1, act=true){
        Convolution(kernel=(kernel,kernel), channels=channels, stride=(stride,stride)) ->
        BatchNorm() ->
        Relu(?=act)
    }
    def resLayer(channels, stride=1, addSkipConv=false){
        (
            conv(kernel=1, channels=channels, stride=stride) ->
            conv(kernel=3, channels=channels) ->
            conv(kernel=1, channels=4*channels, act=false)
        |
            conv(kernel=1, channels=4*channels, stride=stride, act=false, ? = addSkipConv)
        ) ->
        Add() ->
        Relu()
    }

    data ->
    conv(kernel=7, channels=64, stride=2) ->
    Pooling(pool_type="max", kernel=(3,3), stride=(2,2)) ->
    resLayer(channels=64, addSkipConv=true) ->
    resLayer(channels=64, ->=2) ->
    resLayer(channels=128, stride=2, addSkipConv=true) ->
    resLayer(channels=128, ->=7) ->
    resLayer(channels=256, stride=2, addSkipConv=true) ->
    resLayer(channels=256, ->=35) ->
    resLayer(channels=512, stride=2, addSkipConv=true) ->
    resLayer(channels=512, ->=2) ->
    GlobalPooling(pool_type="avg") ->
    FullyConnected(units=classes) ->
    Softmax() ->
    predictions;
}
```

## Assignments
We do not require subnetworks to contain actual layers, so we also provide support for assignments, e.g. `input -> output[0];` is a valid subnetwork.

## Constant Nodes
CNNArch supports integer constant nodes that can be used to initialize output ports with integers, e.g. `1 -> output[0];`.

## Layer Instances
We allow the definition of layer instances that can be used to refer to the same layer and its internals from two different subnetworks which is useful for architectures that consist of more than one subnetwork such as encoder-decoder architectures.
A layer instance is defined by using the keyword `layer` followed by a layer type and a name, e.g. `layer FullyConnected(units=10) fc;`. 
We can use a layer instance in a subnetwork as `input -> fc -> output[0]` and access its internals by using the dot operator.
For all layers we support the `output` internal which refers to the generated output of the layer and can be used as `fc.output -> Softmax() -> output[1];`.
The RNN layers `RNN`, `LSTM` and `GRU` also support the `state` internal which can be used e.g. in a encoder-decoder architecture to initialize the decoder state to the encoder state as `encoder.state -> decoder.state`.

## Unrolling Recurrent Neural Networks
CNNArch supports the unrolling of RNNs with its `timed` block which introduces a time parameter that allows us to define each step of the unrolling. 
We show such a `timed` block in the following encoder-decoder machine translation architecture. 
First, we put the source sequence into an `Embedding` layer, then into the encoder `GRU`. 
Then, we initialize `target[0]` with 1 which is the index of our start of sequence symbol. 
We need to do it because later in the `timed` block we depend on the previously generated target word. 
After that, we initialize `decoder.state` with `encoder.state`. 
Finally, we arrive at the `timed` block which unrolls from 1 to 30 and uses the previously generated target word `target[t-1]` as input for the subnetwork within the `timed` block. The output is then written to `target[t]`. 
The next step, `t` is incremented and the subnetwork is evaluated with our new time parameter. The unrolling stops after it reaches `max_length`.

```
architecture RNNencdec {
    def input Z(0:29999)^{30} source
    def output Z(0:29999)^{1} target[30]

    layer GRU(units=1000) encoder;

    source ->
    Embedding(output_dim=500) ->
    encoder;

    1 -> target[0];

    layer GRU(units=1000) decoder;

    encoder.state -> decoder.state;

    timed<t> GreedySearch(max_length=30) {
        target[t-1] ->
        Embedding(output_dim=500) ->
        decoder ->
        FullyConnected(units=30000) ->
        ArgMax() ->
        target[t]
    };
}
```

## Predefined Layers
All methods with the exception of *Concatenate*, *Add*, *Get* and *Split* can only handle 1 input stream and have 1 output stream. 
All predefined methods start with a capital letter and all constructed methods have to start with a lowercase letter.

* **FullyConnected(units, no_bias=false, flatten=true)**

  Creates a fully connected layer.
    
  * **units** (integer > 0, required): number of neural units in the output.
  * **no_bias** (boolean, optional, default=false): Whether to disable the bias parameter.
  * **flatten** (boolean, default=true): If true, applies Flatten layer to the input if necessary.
  
* **Convolution(kernel, channels, stride=(1,1), padding="same", no_bias=false)**

  Creates a convolutional layer. Currently, only 2D convolutions are allowed
    
  * **kernel** (integer tuple > 0, required): convolution kernel size: (height, width).
  * **channels** (integer > 0, required): number of convolution filters and number of output channels.
  * **stride** (integer tuple > 0, optional, default=(1,1)): convolution stride: (height, width).
  * **padding** ({"valid", "same", "no_loss"}, optional, default="same"): One of "valid", "same" or "no_loss". "valid" means no padding. "same"   results in padding the input such that the output has the same length as the original input divided by the stride (rounded up). "no_loss" results in minimal padding such that each input is used by at least one filter (identical to "valid" if *stride* equals 1).
  * **no_bias** (boolean, optional, default=false): Whether to disable the bias parameter.

* **Softmax()**

  Applies softmax activation function to the input.
    
* **Tanh()**

  Applies tanh activation function to the input.
    
* **Sigmoid()**

  Applies sigmoid activation function to the input.
    
* **Relu()**

  Applies relu activation function to the input.
    
* **Flatten()**

  Reshapes the input such that height and width are 1. 
  Usually not necessary because the FullyConnected layer applies *Flatten* by default.
    
* **Dropout()**

  Applies dropout operation to input array during training.
    
  * **p** (1 >= float >= 0, optional, default=0.5): Fraction of the input that gets dropped out during training time.
  
* **Pooling(pool_type, kernel, stride=(1,1), padding="same")**

  Performs pooling on the input.
  
  * **pool_type** ({"avg", "max"}, required): Pooling type to be applied.
  * **kernel** (integer tuple > 0, required): convolution kernel size: (height, width).
  * **stride** (integer tuple > 0, optional, default=(1,1)): convolution stride: (height, width).
  * **padding** ({"valid", "same", "no_loss"}, optional, default="same"): One of "valid", "same" or "no_loss". "valid" means no padding. "same"   results in padding the input such that the output has the same length as the original input divided by the stride (rounded up). "no_loss" results in minimal padding such that each input is used by at least one filter (identical to "valid" if *stride* equals 1).

* **GlobalPooling(pool_type)**

  Performs global pooling on the input.
  
  * **pool_type** ({"avg", "max"}, required): Pooling type to be applied.

* **Lrn(nsize, knorm=2, alpha=0.0001, beta=0.75)**

  Applies local response normalization to the input.
  See: [mxnet](https://mxnet.incubator.apache.org/api/python/symbol.html#mxnet.symbol.LRN)
    
  * **nsize** (integer > 0, required): normalization window width in elements.
  * **knorm** (float, optional, default=2): The parameter k in the LRN expression.
  * **alpha** (float, optional, default=0.0001): The variance scaling parameter *alpha* in the LRN expression.
  * **beta** (float, optional, default=0.75): The power parameter *beta* in the LRN expression.

* **BatchNorm(fix_gamma=true)**
    
  Batch normalization.
    
  * **fix_gamma** (boolean, optional, default=true): Fix gamma while training.

* **Concatenate(axis=0)**
    
  Merges multiple input streams into one output stream by concatenation of channels. 
  The axes which are not `axis` must be the same size for all inputs.
  The size of `axis` in the output shape is the sum of the size of `axis` in the shape of the input streams.
  
  * **axis** (2 >= integer >= 0): Concatenates on the `axis`-th axis.
    
* **Add()**
    
  Merges multiple input streams into one output stream by adding them element-wise together. 
  The height, width and the number of channels of all inputs must be identical. 
  The output shape is identical to each input shape.
    
* **Get(index)**

  `Get(index=i)` can be abbreviated with `[i]`. Selects one out of multiple input streams. 
  The single output stream is identical to the selected input. 
  
  * **index** (integer >= 0, required): The zero-based index of the selected input.

* **Split(n)**

  Opposite of *Concatenate*. Handles a single input stream and splits it into *n* output streams. 
  The output streams have the same height and width as the input stream and a number channels which is in general `input_channels / n`. 
  The last output stream will have a higher number of channels than the other if `input_channels` is not divisible by `n`.

  * **n** (integer > 0, required): The number of output streams. Cannot be higher than the number of input channels.


* **Embedding(input_dim, output_dim)**

  Creates an embedding layer that maps non-negative integers to dense vectors of fixed size. Input: `Z(0:input_dim - 1)^{N}`, output: `Q^{N, output_dim}`.
  
  * **input_dim** (integer > 0, optional): Size of the vocabulary; optional if it can be inferred from previous layers.
  
  * **output_dim** (integer > 0, required): Dimension of the embedding.
  
  
* **RNN(units, layers=1, dropout=0, bidirectional=false)**

  Creates an RNN layer with tanh as activation function. Input: `Q^{N, S}`, output: `Q^{N, layers * units}`.
  
  * **units** (integer > 0, required): The number of neural units in the hidden state.
  
  * **layers** (integer > 0): The number of recurrent layers.
  
  * **dropout** (1 >= float >= 0): If set, introduces a dropout layer on the outputs of each layer except the last layer. The dropout layer has the specified value as the probability.
  
  * **bidirectional** (boolean): If true, becomes a bidirectional RNN and output changes to `Q^{N, 2 * layers * units}`.
  
  
* **LSTM(units, layers=1, dropout=0, bidirectional=false)**

  Creates an LSTM layer. Input: `Q^{N, S}`, output: `Q^{N, layers * units}`.
  
  * **units** (integer > 0, required): The number of neural units in the hidden state.
  
  * **layers** (integer > 0): The number of recurrent layers.
  
  * **dropout** (1 >= float >= 0): If set, introduces a dropout layer on the outputs of each layer except the last layer. The dropout layer has the specified value as the probability.
  
  * **bidirectional** (boolean): If true, becomes a bidirectional RNN and output changes to `Q^{N, 2 * layers * units}`.
  
  
* **GRU(units, layers=1, dropout=0, bidirectional=false)**

  Creates a GRU layer. Input: `Q^{N, S}`, output: `Q^{N, layers * units}`.
  
  * **units** (integer > 0, required): The number of neural units in the hidden state.
  
  * **layers** (integer > 0): The number of recurrent layers.
  
  * **dropout** (1 >= float >= 0): If set, introduces a dropout layer on the outputs of each layer except the last layer. The dropout layer has the specified value as the probability.
  
  * **bidirectional** (boolean): If true, becomes a bidirectional RNN and output changes to `Q^{N, 2 * layers * units}`.
  
  
* **ExpandDims(axis)**

  Inserts a new dimension of size 1 into the input.
  
  * **axis** (1 >= integer >= 0, required): Position in which the new dimension is inserted.


* **Squeeze(axis)**

  Removes dimensions of size 1 from the input.
  
  * **axis** (2 >= integer >= 0, optional): When specified, only removes the `axis`-th dimension instead of all.


* **Repeat(n, axis)**

  Inserts a new dimension in the front of the input and repeats it.
  
  * **n** (integer > 0, required): The number of repetitions.
  
  * **axis** (2 >= integer >= 0, optional): When specified, repeats on the `axis`-th axis instead of inserting a new dimension.


* **OneHot(size)**

  Creates a OneHot vector of a given size, given a scalar in the previous layer that determines the OneHot-Index (the index at which the *1* in the vector will be placed).

  * **size** (integer > 0, optional): The OneHot-vector's size. Can be omitted to automatically use the output size of the architecture.
 
  
* **ArgMax()**

  Computes the index of the maximal value of its input vector. Useful for recurrent networks, when the output of a timestep should be used as integer input for the next timestep. Is only allowed as last layer of a subnetwork.
  Notice that the Argmax Layer is applied after calculating the loss in the respective backend. This means that loss can still be computed correctly (e.g. from a Softmax layer before the ArgMax), but recurrent networks get only the element with the highest probability as input for their next timestep. Input: `Q^{N}`, output: `Z(0:N - 1)^{1}`.
    
    
* **BroadcastAdd()**

  Takes multiple tensors as input, and broadcasts them to the same shape (Copies values along one axis until it has the size of the largest axis along all inputs). Then performs elementswise addition.


* **BroadcastMultiply()**

  Takes multiple tensors as input, and broadcasts them to the same shape (Copies values along one axis until it has the size of the largest axis along all inputs). Then performs elementswise multiplication.

  
* **Dot()**

  Calculates the dot product of the two inputs. Inputs: `Q^{A, B}` and `Q^{B, C}`, output: `Q^{A, C}`.


* **ExpandDims(axis)**

  Creates a new, empty axis for a given input tensor.

  * **axis** (0 <= integer <= 1, required): The axis to expand.


* **ReduceSum(axis)**

  Sums all values along a given axis, and reduces the dimension of the axis afterwards, making a scalar out of a one-entry vector etc.

  * **axis** (0 <= integer <= 1, optional, default=-1): The axis to sum over. Uses the last axis (-1) by default.


* **Repeat(n, axis)**

  Copies the entries of an axis n times in the same axis.

  * **n** (integer > 0, required): How often to copy the entries of the given axis
  * **axis** (-1 <= integer <= 2, optional, default=-1): The axis to use for copying. Uses the last axis (-1) by default.

* **Reshape(shape)**

  Transforms the input tensor into a different shape, while keeping the number of total entries in the tensor. 

  * **shape** (integer tuple, required): New shape of the tensor.


* **UpConvolution(kernel, channels, stride=(1,1), no_bias=false, padding="same")**

  Creates a up convolutional layer (also known as transposed convolution ). Is currently only supported in the tesnsorflow backend.
    
  * **kernel** (integer tuple > 0, required): convolution kernel size: (height, width).
  * **channels** (integer > 0, required): number of up convolution filters and number of output channels.
  * **stride** (integer tuple > 0, optional, default=(1,1)): up convolution stride: (height, width).
  * **padding** ({"valid", "same", "no_loss"}, optional, default="same"): One of "valid", "same" or "no_loss". "valid" means no padding. "same"   results in padding the input such that the output has the same length as the original input divided by the stride (rounded up). "no_loss" results in minimal padding such that each input is used by at least one filter (identical to "valid" if *stride* equals 1).
  * **no_bias** (boolean, optional, default=false): Whether to disable the bias parameter.
    

## Predefined Unroll Types
* **GreedySearch(max_length)**

  Uses Greedysearch as search algorithm over the timesteps of the RNN, so that only the best output for each timestep is considered.

  * **max_length** (integer > 0, required): The maximum number of timesteps to run the RNN, and thus the maximum length of the generated sequence.

  
* **BeamSearch(max_length, width)**

  Must be used together with a recurrent network. Uses Beamsearch as search algorithm over the timesteps of the RNN.

  * **max_length** (integer > 0, required): The maximum number of timesteps to run the RNN, and thus the maximum length of the generated sequence.
  * **width** (integer > 0, required): The number of candidates to consider each in timestep. Sometimes called k.
