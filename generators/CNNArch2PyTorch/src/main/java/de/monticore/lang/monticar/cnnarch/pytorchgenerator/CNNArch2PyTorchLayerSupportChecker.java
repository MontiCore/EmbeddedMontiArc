package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.monticore.lang.monticar.cnnarch.generator.LayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;

public class CNNArch2PyTorchLayerSupportChecker extends LayerSupportChecker{

    public CNNArch2PyTorchLayerSupportChecker() {
        supportedLayerList.add(AllPredefinedLayers.FULLY_CONNECTED_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONVOLUTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.SOFTMAX_NAME);
        supportedLayerList.add(AllPredefinedLayers.SIGMOID_NAME);
        supportedLayerList.add(AllPredefinedLayers.RELU_NAME);
        supportedLayerList.add(AllPredefinedLayers.DROPOUT_NAME);
        supportedLayerList.add(AllPredefinedLayers.POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.GLOBAL_POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.LRN_NAME);
        supportedLayerList.add(AllPredefinedLayers.BATCHNORM_NAME);
        supportedLayerList.add(AllPredefinedLayers.SPLIT_NAME);
        supportedLayerList.add(AllPredefinedLayers.GET_NAME);
        supportedLayerList.add(AllPredefinedLayers.ADD_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONCATENATE_NAME);
        supportedLayerList.add(AllPredefinedLayers.GRAPH_CONV_NAME);
        supportedLayerList.add(AllPredefinedLayers.LOAD_NETWORK_NAME);
    }
}
