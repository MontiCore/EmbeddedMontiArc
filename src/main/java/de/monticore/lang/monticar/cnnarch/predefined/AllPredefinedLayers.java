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
import de.monticore.lang.monticar.cnnarch._symboltable.UnrollDeclarationSymbol;

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
    public static final String LEAKY_RELU_NAME = "LeakyRelu";
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
    public static final String BEAMSEARCH_NAME = "BeamSearch";
    public static final String GREEDYSEARCH_NAME = "GreedySearch";
    public static final String RNN_NAME = "RNN";
    public static final String LSTM_NAME = "LSTM";
    public static final String GRU_NAME = "GRU";
    public static final String EMBEDDING_NAME = "Embedding";
    public static final String IMG_RESIZE_NAME = "ImgResize";
    public static final String ARG_MAX_NAME = "ArgMax";
    public static final String DOT_NAME = "Dot";
    public static final String REPEAT_NAME = "Repeat";
    public static final String SQUEEZE_NAME = "Squeeze";
    public static final String REDUCE_SUM_NAME = "ReduceSum";
    public static final String EXPAND_DIMS_NAME = "ExpandDims";
    public static final String BROADCAST_MULTIPLY_NAME = "BroadcastMultiply";
    public static final String SWAPAXES_NAME = "SwapAxes";
    public static final String BROADCAST_ADD_NAME = "BroadcastAdd";
    public static final String RESHAPE_NAME = "Reshape";
    public static final String MEMORY_NAME = "Memory";

    //predefined argument names
    public static final String KERNEL_NAME = "kernel";
    public static final String CHANNELS_NAME = "channels";
    public static final String STRIDE_NAME = "stride";
    public static final String GROUPS_NAME= "groups";
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
    public static final String TRANSPADDING_NAME = "padding";
    public static final String POOL_TYPE_NAME = "pool_type";
    public static final String SIZE_NAME = "size";
    public static final String LAYERS_NAME = "layers";
    public static final String INPUT_DIM_NAME = "input_dim";
    public static final String OUTPUT_DIM_NAME = "output_dim";
    public static final String BIDIRECTIONAL_NAME = "bidirectional";
    public static final String FLATTEN_PARAMETER_NAME = "flatten";
    public static final String MAX_LENGTH_NAME = "max_length";
    public static final String WIDTH_NAME = "width";
    public static final String REPEATS_NAME = "n";
    public static final String AXIS_NAME = "axis";
    public static final String AXES_NAME = "axes";
    public static final String BEAMSEARCH_MAX_LENGTH = "max_length";
    public static final String BEAMSEARCH_WIDTH_NAME = "width";
    public static final String SHAPE_NAME = "shape";
    public static final String RNN_DROPOUT_NAME = "dropout";
	//parameters for memory layers
    public static final String SUB_KEY_SIZE_NAME = "subKeySize";
    public static final String QUERY_SIZE_NAME = "querySize";
	public static final String ACT_QUERY_NAME = "actQuery";
    public static final String K_NAME = "k";
	public static final String NUM_HEADS_NAME = "numHeads";

    //possible String values
    public static final String PADDING_VALID = "valid";
    public static final String PADDING_SAME = "same";
    public static final String PADDING_NO_LOSS = "no_loss";
    public static final String POOL_MAX = "max";
    public static final String POOL_AVG = "avg";
	
	//possible activation values for the querry network in the memory layer
	public static final String MEMORY_ACTIVATION_LINEAR = "linear";
	public static final String MEMORY_ACTIVATION_RELU = "relu";
    public static final String MEMORY_ACTIVATION_TANH = "tanh";
	public static final String MEMORY_ACTIVATION_SIGMOID = "sigmoid";
	public static final String MEMORY_ACTIVATION_SOFTRELU = "softrelu";
    public static final String MEMORY_ACTIVATION_SOFTSIGN = "softsign";

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
				LeakyRelu.create(),
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
                ArgMax.create(),
                Dot.create(),
                Repeat.create(),
                Squeeze.create(),
                ReduceSum.create(),
                ExpandDims.create(),
                BroadcastMultiply.create(),
                SwapAxes.create(),
                BroadcastAdd.create(),
                Reshape.create(),
                Memory.create());
    }

    public static List<UnrollDeclarationSymbol> createUnrollList(){
        return Arrays.asList(
                GreedySearch.create(),
                BeamSearch.create());
    }
}
