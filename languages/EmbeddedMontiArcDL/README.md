<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArcDL/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/EmbeddedMontiArcDL/badges/master/coverage.svg)

# EmbeddedMontiArcDL
Embeds [CNNArch](https://github.com/EmbeddedMontiArc/CNNArchLang), [CNNTrain](https://github.com/EmbeddedMontiArc/CNNTrainLang) and MontiMath into EmbeddedMontiArc.

## Examples
In the following, we list common CNN architectures that are modeled inside an EMA component.
```
component LeNet{
    ports in Z(0:255)^{1,28,28} data,
          out Q(0:1)^{10} predictions;

    implementation CNN {
        data ->
        Convolution(kernel=(5,5), channels=20) ->
        Tanh() ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        Convolution(kernel=(5,5), channels=50) ->
        Tanh() ->
        Pooling(pool_type="max", kernel=(2,2), stride=(2,2)) ->
        FullyConnected(units=500) ->
        Tanh() ->
        Dropout() ->
        FullyConnected(units=10) ->
        Softmax() ->
        predictions
    }
}
```
```
component VGG16<Z(1:oo) channels=3, Z(1:oo) height=224, Z(1:oo) width=224, Z(2:oo) classes=1000>{
    ports in Z(0:255)^{channels, height, width} image,
         out Q(0:1)^{classes} predictions;

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
        FullyConnected(units=classes) ->
        Softmax() ->
        predictions
    }
}
```
```
component ResNet34<Z(1:oo) channels=3, Z(1:oo) height=224, Z(1:oo) width=224, Z(2:oo) classes=1000>{
    ports in Z(0:255)^{channels, height, width} image,
         out Q(0:1)^{classes} predictions;

    implementation CNN {
        def conv(filter, channels, stride=1, act=true){
            Convolution(kernel=(filter,filter), channels=channels, stride=(stride,stride)) ->
            BatchNorm() ->
            Relu(?=act)
        }
        def skip(channels, stride){
            Convolution(kernel=(1,1), channels=channels, stride=(stride,stride)) ->
            BatchNorm()
        }
        def resLayer(channels, stride=1){
            (
                conv(filter=3, channels=channels, stride=stride) ->
                conv(filter=3, channels=channels, act=false)
            |
                skip(channels=channels, stride=stride, ? = (stride != 1))
            ) ->
            Add() ->
            Relu()
        }
    
        image ->
        conv(filter=7, channels=64, stride=2) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2)) ->
        resLayer(channels=64, ->=3) ->
        resLayer(channels=128, stride=2) ->
        resLayer(channels=128, ->=3) ->
        resLayer(channels=256, stride=2) ->
        resLayer(channels=256, ->=5) ->
        resLayer(channels=512, stride=2) ->
        resLayer(channels=512, ->=2) ->
        GlobalPooling(pool_type="avg") ->
        FullyConnected(units=classes) ->
        Softmax() ->
        predictions
    }
}
```
```
component Alexnet<Z(1:oo) channels=3, Z(1:oo) height=224, Z(1:oo) width=224, Z(2:oo) classes=1000>{
    ports in Z(0:255)^{channels, height, width} image,
          out Q(0:1)^{classes} predictions;

    implementation CNN {
        def split1(i){
            [i] ->
            Convolution(kernel=(5,5), channels=128) ->
            Lrn(nsize=5, alpha=0.0001, beta=0.75) ->
            Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
            Relu()
        }
        def split2(i){
            [i] ->
            Convolution(kernel=(3,3), channels=192) ->
            Relu() ->
            Convolution(kernel=(3,3), channels=128) ->
            Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
            Relu()
        }
        def fc(){
            FullyConnected(units=4096) ->
            Relu() ->
            Dropout()
        }
        image ->
        Convolution(kernel=(11,11), channels=96, stride=(4,4), padding="no_loss") ->
        Lrn(nsize=5, alpha=0.0001, beta=0.75) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2), padding="no_loss") ->
        Relu() ->
        Split(n=2) ->
        split1(i=[0|1]) ->
        Concatenate() ->
        Convolution(kernel=(3,3), channels=384) ->
        Relu() ->
        Split(n=2) ->
        split2(i=[0|1]) ->
        Concatenate() ->
        fc(->=2) ->
        FullyConnected(units=classes) ->
        Softmax() ->
        predictions
    }
}
```
```
component ResNeXt50<Z(1:oo) channels=3, Z(1:oo) height=224, Z(1:oo) width=224, Z(2:oo) classes=1000>{
    ports in Z(0:255)^{channels, height, width} image,
         out Q(0:1)^{classes} predictions;

    implementation CNN {
        def conv(kernel, channels, stride=1, act=true){
            Convolution(kernel=(kernel,kernel), channels=channels, stride=(stride,stride)) ->
            BatchNorm() ->
            Relu(?=act)
        }
        def resGroup(innerChannels, outChannels, stride=1){
            conv(kernel=1, channels=innerChannels) ->
            conv(kernel=3, channels=innerChannels, stride=stride) ->
            conv(kernel=1, channels=outChannels, act=false)
        }
        def resLayer(innerChannels, outChannels, stride=1, addSkipConv=false){
            (
                resGroup(innerChannels=innerChannels,
                         outChannels=outChannels,
                         stride=stride,
                         | = 32) ->
                Add()
            |
                conv(kernel=1, channels=outChannels, stride=stride, act=false, ? = addSkipConv)
            ) ->
            Add() ->
            Relu()
        }

        data ->
        conv(kernel=7, channels=64, stride=2) ->
        Pooling(pool_type="max", kernel=(3,3), stride=(2,2)) ->
        resLayer(innerChannels=4, outChannels=256, addSkipConv=true) ->
        resLayer(innerChannels=4, outChannels=256, -> = 2) ->
        resLayer(innerChannels=8, outChannels=512, stride=2, addSkipConv=true) ->
        resLayer(innerChannels=8, outChannels=512, -> = 3) ->
        resLayer(innerChannels=16, outChannels=1024, stride=2, addSkipConv=true) ->
        resLayer(innerChannels=16, outChannels=1024, -> = 5) ->
        resLayer(innerChannels=32, outChannels=2048, stride=2, addSkipConv=true) ->
        resLayer(innerChannels=32, outChannels=2048, -> = 2) ->
        GlobalPooling(pool_type="avg") ->
        FullyConnected(units=classes) ->
        Softmax() ->
        predictions
    }
}
```
