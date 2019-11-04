/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.LayerDeclarationSymbol;

import java.util.Arrays;
import java.util.List;

public class AllPredefinedLayers {

    //predefined layer names
    public static final String FULLY_CONNECTED_NAME = "FullyConnected";
    public static final String CONVOLUTION_NAME = "Convolution";
    public static final String UP_CONVOLUTION_NAME = "UpConvolution";
    public static final String SOFTMAX_NAME = "Softmax";
    public static final String SIGMOID_NAME = "Sigmoid";
    public static final String TANH_NAME = "Tanh";
    public static final String RELU_NAME = "Relu";
    public static final String DROPOUT_NAME = "Dropout";
    public static final String POOLING_NAME = "Pooling";
    public static final String GLOBAL_POOLING_NAME = "GlobalPooling";
    public static final String LRN_NAME = "Lrn";
    public static final String BATCHNORM_NAME = "BatchNorm";
    public static final String SPLIT_NAME = "Split";
    public static final String GET_NAME = "Get";
    public static final String ADD_NAME = "Add";
    public static final String CONCATENATE_NAME = "Concatenate";
    public static final String FLATTEN_NAME = "Flatten";
    public static final String ONE_HOT_NAME = "OneHot";
    public static final String BEAMSEARCH_NAME = "BeamSearchStart";
    public static final String RNN_NAME = "RNN";
    public static final String LSTM_NAME = "LSTM";
    public static final String GRU_NAME = "GRU";
    public static final String EMBEDDING_NAME = "Embedding";
    public static final String RESHAPE_NAME = "Reshape";

    //predefined argument names
    public static final String KERNEL_NAME = "kernel";
    public static final String CHANNELS_NAME = "channels";
    public static final String STRIDE_NAME = "stride";
    public static final String UNITS_NAME = "units";
    public static final String NOBIAS_NAME = "no_bias";
    public static final String P_NAME = "p";
    public static final String INDEX_NAME = "index";
    public static final String NUM_SPLITS_NAME = "n";
    public static final String FIX_GAMMA_NAME = "fix_gamma";
    public static final String NSIZE_NAME = "nsize";
    public static final String KNORM_NAME = "knorm";
    public static final String ALPHA_NAME = "alpha";
    public static final String BETA_NAME = "beta";
    public static final String PADDING_NAME = "padding";
    public static final String POOL_TYPE_NAME = "pool_type";
    public static final String SIZE_NAME = "size";
    public static final String LAYERS_NAME = "layers";
    public static final String INPUT_DIM_NAME = "input_dim";
    public static final String OUTPUT_DIM_NAME = "output_dim";
    public static final String BIDIRECTIONAL_NAME = "bidirectional";
    public static final String FLATTEN_PARAMETER_NAME = "flatten";
    public static final String BEAMSEARCH_MAX_LENGTH = "max_length";
    public static final String BEAMSEARCH_WIDTH_NAME = "width";
    public static final String SHAPE_NAME = "shape";

    //possible String values
    public static final String PADDING_VALID = "valid";
    public static final String PADDING_SAME = "same";
    public static final String PADDING_NO_LOSS = "no_loss";
    public static final String POOL_MAX = "max";
    public static final String POOL_AVG = "avg";


    //list with all predefined layers
    public static List<LayerDeclarationSymbol> createList(){
        return Arrays.asList(
                FullyConnected.create(),
                Convolution.create(),
                UpConvolution.create(),
                Softmax.create(),
                Sigmoid.create(),
                Tanh.create(),
                Relu.create(),
                Dropout.create(),
                Flatten.create(),
                Pooling.create(),
                GlobalPooling.create(),
                Lrn.create(),
                BatchNorm.create(),
                Split.create(),
                Get.create(),
                Add.create(),
                Concatenate.create(),
                OneHot.create(),
                RNN.create(),
                LSTM.create(),
                GRU.create(),
                Embedding.create(),
                Reshape.create()),
                RNN.create());
    }
}
