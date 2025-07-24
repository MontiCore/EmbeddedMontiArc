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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AllPredefinedLayers {

    //predefined layer names
    public static final String FULLY_CONNECTED_NAME = "FullyConnected";
    public static final String CONVOLUTION_NAME = "Convolution";
    public static final String IDENTITY_NAME = "Identity";
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
    public static final String LAYERNORM_NAME = "LayerNorm";
    public static final String SPLIT_NAME = "Split";
    public static final String GET_NAME = "Get";
    public static final String ADD_NAME = "Add";
    public static final String CONCATENATE_NAME = "Concatenate";
    public static final String REPARAMETERIZE_NAME = "Reparameterize";
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
    public static final String DOT_PRODUCT_SELF_ATTENTION_NAME = "DotProductSelfAttention";
    public static final String LOAD_NETWORK_NAME = "LoadNetwork";
    public static final String CUSTOM_LAYER = "CustomLayer";
    public static final String CONVOLUTION3D_NAME = "Convolution3D";
    public static final String UP_CONVOLUTION3D_NAME = "UpConvolution3D";
    public static final String VECTOR_QUANTIZE_NAME = "VectorQuantize";
    public static final String POSITION_ENCODING_NAME = "PositionEncoding";
    public static final String SAC_SQUASHED_GAUSSIAN_NAME = "SACSquashedGaussian";

    public static final String AdaNet_Name = "AdaNet"; //AdaNet layer

    //replay layers
    public static final String LARGE_MEMORY_NAME = "LargeMemory";
    public static final String EPISODIC_MEMORY_NAME = "EpisodicMemory";
    public static final List<String> EPISODIC_REPLAY_LAYER_NAMES = new ArrayList<String>(Arrays.asList(EPISODIC_MEMORY_NAME));

    //DGL layers
    public static final String GRAPH_CONV_NAME = "GraphConv";
    public static final String GAT_CONV_NAME = "GATConv";
    public static final String GRAPH_AVG_POOL_NAME = "GraphAvgPool";
    public static final String GRAPH_SUM_POOL_NAME = "GraphSumPool";

    //predefined argument names
    public static final String CUSTOM_PARAMETERS = "parameters";
    public static final String CUSTOM_NAME = "customname";
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

    //Parameter AdaNet
    public static final String Block = "block";
    public static final String In = "inBlock";
    public static final String Out = "outBlock";
    public static final Integer DEFAULT_UNITS = 20;
    public static final String DEFAULT_BLOCK = "default_block";

    //VAE Parameters
    public static final String NUM_EMBEDDINGS_NAME = "num_embeddings";
    public static final String EMA_NAME = "ema";

    // SACSquashedGaussian parameters
    public static final String SAC_STD_LOG_MIN_NAME = "std_log_min";
    public static final String SAC_STD_LOG_MAX_NAME = "std_log_max";

    //parameters LoadNetwork layer
    public static final String NETWORK_DIR_NAME = "networkDir";
    public static final String NETWORK_PREFIX_NAME = "networkPrefix";
    public static final String NUM_INPUTS_NAME = "numInputs";
    public static final String OUTPUT_SHAPE_NAME = "outputShape";
    public static final String TRAINABLE = "trainable";
    public static final String ELEMENT_TYPE_LOWER_BOUND = "elementTypeLowerBound";
    public static final String ELEMENT_TYPE_UPPER_BOUND = "elementTypeUpperBound";


    //parameters DotProductSelfAttention
    public static final String SCALE_FACTOR_NAME="scaleFactor";
    public static final String DIM_KEYS_NAME="dimKeys";
    public static final String DIM_VALUES_NAME="dimValues";
    public static final String USE_PROJ_BIAS_NAME="useProjBias";
    public static final String USE_MASK_NAME="useMask";

    //shared parameters episodic replay layers
    public static final String USE_REPLAY_NAME = "useReplay";
    public static final String REPLAY_INTERVAL_NAME = "replayInterval";
    public static final String REPLAY_BATCH_SIZE_NAME = "replayBatchSize";
    public static final String REPLAY_STEPS_NAME = "replaySteps";
    public static final String REPLAY_GRADIENT_STEPS_NAME = "replayGradientSteps";
    public static final String USE_LOCAL_ADAPTATION_NAME = "useLocalAdaptation";
    public static final String LOCAL_ADAPTATION_K_NAME = "localAdaptationK";
    public static final String LOCAL_ADAPTATION_GRADIENT_STEPS_NAME = "localAdaptationGradientSteps";
    public static final String MAX_STORED_SAMPLES_NAME = "maxStoredSamples";
    public static final String MEMORY_REPLACEMENT_STRATEGY_NAME = "memoryReplacementStrategy";
    public static final String MEMORY_STORE_PROB_NAME = "memoryStoreProb";
    public static final String QUERY_NET_DIR_NAME = "queryNetDir";
    public static final String QUERY_NET_PREFIX_NAME = "queryNetPrefix";
    public static final String QUERY_NET_NUM_INPUTS_NAME = "queryNetNumInputs";

	//parameters for large memory layer
    public static final String SUB_KEY_SIZE_NAME = "subKeySize";
    public static final String QUERY_SIZE_NAME = "querySize";
	public static final String QUERY_ACT_NAME = "queryAct";
    public static final String K_NAME = "k";
	public static final String NUM_HEADS_NAME = "numHeads";
	public static final String VALUES_DIM_NAME = "valuesDim";

    //parameters for dgl layers
    public static final String NODES_NAME = "nodes";

    //possible String values
    public static final String PADDING_VALID = "valid";
    public static final String PADDING_SAME = "same";
    public static final String PADDING_NO_LOSS = "no_loss";
    public static final String PADDING_VALID3D = "valid3d";
    public static final String PADDING_SAME3D = "same3d";
    public static final String PADDING_SIMPLE3D = "simple3d";
    public static final String POOL_MAX = "max";
    public static final String POOL_AVG = "avg";
    public static final String RANDOM = "random";
    public static final String REPLACE_OLDEST = "replace_oldest";
    public static final String NO_REPLACEMENT = "no_replacement";

    // String values for Reparametrization Layer
    public static final String PDF_NAME = "pdf";
    public static final String PDF_NORMAL = "normal";
    public static final String PDF_DIRICHLET = "dirichlet";

	//possible activation values for the querry network in the memory layer
	public static final String MEMORY_ACTIVATION_LINEAR = "linear";
	public static final String MEMORY_ACTIVATION_RELU = "relu";
    public static final String MEMORY_ACTIVATION_TANH = "tanh";
	public static final String MEMORY_ACTIVATION_SIGMOID = "sigmoid";
	public static final String MEMORY_ACTIVATION_SOFTRELU = "softrelu";
    public static final String MEMORY_ACTIVATION_SOFTSIGN = "softsign";


    public static List<String> getLossParameterizingLayers() {
        return Arrays.asList(
                REPARAMETERIZE_NAME,
                VECTOR_QUANTIZE_NAME
        );
    }


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
                LayerNorm.create(),
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
                LoadNetwork.create(),
                DotProductSelfAttention.create(),
                LargeMemory.create(),
                EpisodicMemory.create(),
                Convolution3D.create(),
                UpConvolution3D.create(),
                GraphConv.create(),
                GATConv.create(),
                GraphAvgPool.create(),
                GraphSumPool.create(),
                AdaNet.create(),
                Reparameterize.create(),
                VectorQuantize.create(),
                SACSquashedGaussian.create(),
                PositionEncoding.create(),
                Identity.create());

    }

    public static List<UnrollDeclarationSymbol> createUnrollList(){
        return Arrays.asList(
                GreedySearch.create(),
                BeamSearch.create());
    }
}
