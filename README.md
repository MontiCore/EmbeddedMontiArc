![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP/badges/master/coverage.svg)
# EMADL2CPP
Generates CPP/Python code for EmbeddedMontiArcDL.
See example project [EMADL-Demo](https://git.rwth-aachen.de/thomas.timmermanns/EMADL-Demo) for more information on how the generated code can be used.



[ How to develop and train a CNN component using EMADL2CPP](#nn)
<a name="nn"></a>
# Development and training of a CNN component using EMADL2CPP

## Prerequisites
* Linux. Ubuntu Linux 16.04 and 18.04 were used during testing. The docker files can be found [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP/tree/oneclick_nn_training/src/test/resources/docker) 
* Deep learning backend:
    * MXNet
        * training - generated is Python code. Required is Python 2.7 or higher, Python packages `h5py`, `mxnet` (for training on CPU) or e.g. `mxnet-cu75` for CUDA 7.5 (for training on GPU with CUDA, concrete package should be selected according to CUDA version). Follow [official instructions on MXNet site](https://mxnet.incubator.apache.org/install/index.html?platform=Linux&language=Python&processor=CPU)
        * prediction - generated code is C++. Install MXNet using [official instructions on MXNet site](https://mxnet.incubator.apache.org) for C++.
     
     * Caffe2
        * training - generated is Python code. Follow [ official instructions on Caffe2 site ](https://caffe2.ai/docs/getting-started.html?platform=ubuntu&configuration=prebuilt)
     * Gluon

### HowTo
1. Define a EMADL component containing architecture of a neural network and save it in a `.emadl` file. For more information on architecture language please refer to [CNNArchLang project](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNArchLang). An example of NN architecture:
```
component VGG16{
    ports in Z(0:255)^{3, 224, 224} image,
         out Q(0:1)^{1000} predictions;

    implementation CNN {

        def conv(filter, channels){
            Convolution(kernel=(filter,filter), channels=channels) ->
            Relu()
        }
        def fc(){
            FullyConnected(units=4096) ->
            Relu() ->
            Dropout(p=0.5)
        }

        image ->
        conv(filter=3, channels=64, ->=2) ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        conv(filter=3, channels=128, ->=2) ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        conv(filter=3, channels=256, ->=3) ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        conv(filter=3, channels=512, ->=3) ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        conv(filter=3, channels=512, ->=3) ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        fc() ->
        fc() ->
        FullyConnected(units=1000) ->
        Softmax() ->
        predictions
    }
}
```
2. Define a training configuration for this network and store it in a `.cnnt file`, the name of the file should be the same as that of the corresponding architecture (e.g. `VGG16.emadl` and `VGG16.cnnt`). For more information on architecture language please refer to [CNNTrainLang project](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNTrainLang). An example of a training configuration:
```
configuration VGG16{
    num_epoch:10
    batch_size:64
    normalize:true
    load_checkpoint:false
    optimizer:adam{
        learning_rate:0.01
        learning_rate_decay:0.8
        step_size:1000
    }
}
```
3. Generate C code which uses neural networks that were trained using the specified deep learning backend. The generator receives the following command line parameters:
    * `-m`    path to directory with EMADL models
    * `-r`    name of the root model
    * `-o`    output path
    * `-b`    backend
    * `-p`   path to python (Not mandatory; Default is `/usr/bin/python`)
    * `-f`   forced training (Not mandatory; values can be `y` for a forced training and `n` for a skip (a forced no-training)). By default, the hash value (from the training and test data, the structure of the model (.emadl) and the training parameters (.cnnt) of the model) will be compared. The model is retrained only if the hash changes. This can be used to distribute trained models, by distributing the corresponding `.training_hash` file as well, which will prevent a retraining
    * `-c`    compiling of generated c code (Not mandators; Default is `y`). Disable by setting to `n` when running on Windows, or on machines without `make` and `cmake` commands

    Assuming both the architecture definition `VGG16.emadl` and the corresponding training configuration `VGG16.cnnt` are located in a folder `models` and the target code should be generated in  a `target` folder using the `MXNet` backend, an example of a command is then:  
    ```java -jar embedded-montiarc-emadl-generator-0.2.10-jar-with-dependencies.jar -m models -r VGG16 -o target -b MXNET```
