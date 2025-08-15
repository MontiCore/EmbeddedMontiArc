/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.cnnarch.generator.LayerSupportChecker;

public class CNNArch2GluonLayerSupportChecker extends LayerSupportChecker {

    public CNNArch2GluonLayerSupportChecker() {
        supportedLayerList.add(AllPredefinedLayers.REPARAMETERIZE_NAME);
        supportedLayerList.add(AllPredefinedLayers.FULLY_CONNECTED_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONVOLUTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.UP_CONVOLUTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.SOFTMAX_NAME);
        supportedLayerList.add(AllPredefinedLayers.SIGMOID_NAME);
        supportedLayerList.add(AllPredefinedLayers.TANH_NAME);
        supportedLayerList.add(AllPredefinedLayers.RELU_NAME);
        supportedLayerList.add(AllPredefinedLayers.LEAKY_RELU_NAME);
        supportedLayerList.add(AllPredefinedLayers.DROPOUT_NAME);
        supportedLayerList.add(AllPredefinedLayers.POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.GLOBAL_POOLING_NAME);
        supportedLayerList.add(AllPredefinedLayers.LRN_NAME);
        supportedLayerList.add(AllPredefinedLayers.BATCHNORM_NAME);
        supportedLayerList.add(AllPredefinedLayers.SPLIT_NAME);
        supportedLayerList.add(AllPredefinedLayers.GET_NAME);
        supportedLayerList.add(AllPredefinedLayers.ADD_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONCATENATE_NAME);
        supportedLayerList.add(AllPredefinedLayers.FLATTEN_NAME);
        supportedLayerList.add(AllPredefinedLayers.ONE_HOT_NAME);
        supportedLayerList.add(AllPredefinedLayers.RNN_NAME);
        supportedLayerList.add(AllPredefinedLayers.LSTM_NAME);
        supportedLayerList.add(AllPredefinedLayers.GRU_NAME);
        supportedLayerList.add(AllPredefinedLayers.EMBEDDING_NAME);
        supportedLayerList.add(AllPredefinedLayers.ARG_MAX_NAME);
        supportedLayerList.add(AllPredefinedLayers.REPEAT_NAME);
        supportedLayerList.add(AllPredefinedLayers.DOT_NAME);
        supportedLayerList.add(AllPredefinedLayers.EXPAND_DIMS_NAME);
        supportedLayerList.add(AllPredefinedLayers.SQUEEZE_NAME);
        supportedLayerList.add(AllPredefinedLayers.SWAPAXES_NAME);
        supportedLayerList.add(AllPredefinedLayers.BROADCAST_MULTIPLY_NAME);
        supportedLayerList.add(AllPredefinedLayers.REDUCE_SUM_NAME);
        supportedLayerList.add(AllPredefinedLayers.BROADCAST_ADD_NAME);
        supportedLayerList.add(AllPredefinedLayers.RESHAPE_NAME);
        // supportedLayerList.add(AllPredefinedLayers.CROP_NAME);
        supportedLayerList.add(AllPredefinedLayers.LARGE_MEMORY_NAME);
		supportedLayerList.add(AllPredefinedLayers.EPISODIC_MEMORY_NAME);
        supportedLayerList.add(AllPredefinedLayers.DOT_PRODUCT_SELF_ATTENTION_NAME);
        supportedLayerList.add(AllPredefinedLayers.LOAD_NETWORK_NAME);
        supportedLayerList.add(AllPredefinedLayers.LAYERNORM_NAME);
        supportedLayerList.add(AllPredefinedLayers.UP_CONVOLUTION3D_NAME);
        supportedLayerList.add(AllPredefinedLayers.CONVOLUTION3D_NAME);
        supportedLayerList.add(AllPredefinedLayers.VECTOR_QUANTIZE_NAME);
        supportedLayerList.add(AllPredefinedLayers.GRAPH_CONV_NAME);
        supportedLayerList.add(AllPredefinedLayers.GAT_CONV_NAME);
        supportedLayerList.add(AllPredefinedLayers.GRAPH_AVG_POOL_NAME);
        supportedLayerList.add(AllPredefinedLayers.GRAPH_SUM_POOL_NAME);
        supportedLayerList.add(AllPredefinedLayers.AdaNet_Name);
        supportedLayerList.add(AllPredefinedLayers.POSITION_ENCODING_NAME);
        supportedLayerList.add(AllPredefinedLayers.IDENTITY_NAME);
    }

}
