/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.LayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;

public class CNNArch2MxNetLayerSupportChecker extends LayerSupportChecker {

    public CNNArch2MxNetLayerSupportChecker() {
        supportedLayerList.add(AllPredefinedLayers.FULLY_CONNECTED_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONVOLUTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.SOFTMAX_NAME);
        supportedLayerList.add(AllPredefinedLayers.SIGMOID_NAME);
        supportedLayerList.add(AllPredefinedLayers.TANH_NAME);
        supportedLayerList.add(AllPredefinedLayers.RELU_NAME);
        supportedLayerList.add(AllPredefinedLayers.DROPOUT_NAME);
        supportedLayerList.add(AllPredefinedLayers.POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.EMBEDDING_NAME);
        supportedLayerList.add(AllPredefinedLayers.GLOBAL_POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.LRN_NAME);
        supportedLayerList.add(AllPredefinedLayers.BATCHNORM_NAME);
        supportedLayerList.add(AllPredefinedLayers.SPLIT_NAME);
        supportedLayerList.add(AllPredefinedLayers.GET_NAME);
        supportedLayerList.add(AllPredefinedLayers.ADD_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONCATENATE_NAME);
        supportedLayerList.add(AllPredefinedLayers.FLATTEN_NAME);
        supportedLayerList.add(AllPredefinedLayers.ONE_HOT_NAME);
        supportedLayerList.add(AllPredefinedLayers.RESHAPE_NAME);
    }

}
