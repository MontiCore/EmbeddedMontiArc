/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ConstantSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerVariableDeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
import java.util.*;

public class ArchitectureElementData {

    private String name;
    private ArchitectureElementSymbol element;
    private CNNArchTemplateController templateController;

    public ArchitectureElementData(String name, ArchitectureElementSymbol element, CNNArchTemplateController templateController) {
        this.name = name;
        this.element = element;
        this.templateController = templateController;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public ArchitectureElementSymbol getElement() {
        return element;
    }

    public void setElement(ArchitectureElementSymbol element) {
        this.element = element;
    }

    public CNNArchTemplateController getTemplateController() {
        return templateController;
    }

    public void setTemplateController(CNNArchTemplateController templateController) {
        this.templateController = templateController;
    }

    private LayerSymbol getLayerSymbol() {
        if (getElement() instanceof VariableSymbol) {
            return ((VariableSymbol) getElement()).getLayerVariableDeclaration().getLayer();
        }
        else {
            assert getElement() instanceof LayerSymbol;

            return (LayerSymbol) getElement();
        }
    }

    public boolean isVariable() {
        return getElement() instanceof VariableSymbol;
    }

    public List<String> getInputs(){
        return getTemplateController().getLayerInputs(getElement());
    }

    public String getMember() {
        if (getElement() instanceof VariableSymbol) {
            return ((VariableSymbol) getElement()).getMember().toString();
        }
        else {
            return VariableSymbol.Member.NONE.toString();
        }

    }

    public List<Integer> getKernel(){
        return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
    }

    public int getChannels(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.CHANNELS_NAME).get();
    }

    public List<Integer> getStride(){
        return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
    }

    public String getPdf() { return getLayerSymbol().getStringValue(AllPredefinedLayers.PDF_NAME).get(); }

    public int getNumEmbeddings() { return getLayerSymbol().getIntValue(AllPredefinedLayers.NUM_EMBEDDINGS_NAME).get();
}
    public int getGroups(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.GROUPS_NAME).get();
    }

    public int getUnits(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.UNITS_NAME).get();
    }

    public boolean getNoBias(){
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.NOBIAS_NAME).get();
    }

    public double getP(){
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.P_NAME).get();
    }

    public int getIndex(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.INDEX_NAME).get();
    }

    public int getNumOutputs(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.NUM_SPLITS_NAME).get();
    }

    public boolean getFixGamma(){
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.FIX_GAMMA_NAME).get();
    }

    public int getNsize(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.NSIZE_NAME).get();
    }

    public double getKnorm(){
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.KNORM_NAME).get();
    }

    public double getAlpha(){
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.ALPHA_NAME).get();
    }

    public double getBeta(){
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.BETA_NAME).get();
    }

    public int getSize(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.SIZE_NAME).get();
    }

    public int getRepeats(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.REPEATS_NAME).get();
    }

    public int getAxis(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.AXIS_NAME).get();
    }

    public List<Integer> getAxes(){
        return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.AXES_NAME).get();
    }

    public List<Integer> getShape(){
        return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.SHAPE_NAME).get();
    }

    public int getLayers(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.LAYERS_NAME).get();
    }

    public int getInputDim(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.INPUT_DIM_NAME).get();
    }

    public int getOutputDim(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.OUTPUT_DIM_NAME).get();
    }

    public boolean getBidirectional() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.BIDIRECTIONAL_NAME).get();
    }

    public double getDropout() {
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.RNN_DROPOUT_NAME).get();
    }

    public boolean getFlatten() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.FLATTEN_PARAMETER_NAME).get();
    }

    @Nullable
    public String getPoolType(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.POOL_TYPE_NAME).get();
    }

    public String getNetworkDir(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.NETWORK_DIR_NAME).get();
    }

    public String getNetworkPrefix(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.NETWORK_PREFIX_NAME).get();
    }
    public String getElementTypeLowerBound(){return getLayerSymbol().getStringValue(AllPredefinedLayers.ELEMENT_TYPE_LOWER_BOUND).get();}
    public String getElementTypeUpperBound(){return getLayerSymbol().getStringValue(AllPredefinedLayers.ELEMENT_TYPE_UPPER_BOUND).get();}
    public int getNumInputs(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.NUM_INPUTS_NAME).get();
    }

    public boolean getTrainable(){
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.TRAINABLE).get();
    }

    public List<Integer> getOutputShape(){
        if (getLayerSymbol().getIntValue(AllPredefinedLayers.OUTPUT_SHAPE_NAME).isPresent()){
            List<Integer> list = new ArrayList<>();
            list.add((Integer) getLayerSymbol().getIntValue(AllPredefinedLayers.OUTPUT_SHAPE_NAME).get());
            return list;
        }else{
            return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.OUTPUT_SHAPE_NAME).get();
        }
    }

    public int getScaleFactor(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.SCALE_FACTOR_NAME).get();
    }

    public int getDimKeys(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.DIM_KEYS_NAME).get();
    }

    public int getDimValues(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.DIM_VALUES_NAME).get();
    }

    public boolean getUseProjBias() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.USE_PROJ_BIAS_NAME).get();
    }

    public boolean getUseMask() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.USE_MASK_NAME).get();
    }

    public int getSubKeySize(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.SUB_KEY_SIZE_NAME).get();
    }
    
    public List<Integer> getQuerySize(){   
        if (getLayerSymbol().getIntValue(AllPredefinedLayers.QUERY_SIZE_NAME).isPresent()){
            List<Integer> list = new ArrayList<>();
            list.add((Integer) getLayerSymbol().getIntValue(AllPredefinedLayers.QUERY_SIZE_NAME).get());
            return list;
        }else{
            return getLayerSymbol().getIntTupleValue(AllPredefinedLayers.QUERY_SIZE_NAME).get();
        }
    }
	
    public String getQueryAct(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.QUERY_ACT_NAME).get();
    }
    
    public int getK(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.K_NAME).get();
    }
	
	public int getNumHeads(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.NUM_HEADS_NAME).get();
    }
    
    public int getReplayInterval(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.REPLAY_INTERVAL_NAME).get();
    }
    
    public int getReplayBatchSize(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.REPLAY_BATCH_SIZE_NAME).get();
    }
    
    public int getReplaySteps(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.REPLAY_STEPS_NAME).get();
    }
    
    public int getReplayGradientSteps(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.REPLAY_GRADIENT_STEPS_NAME).get();
    }
    
    public double getMemoryStoreProb(){
        return getLayerSymbol().getDoubleValue(AllPredefinedLayers.MEMORY_STORE_PROB_NAME).get();
    }
    
    public int getMaxStoredSamples(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.MAX_STORED_SAMPLES_NAME).get();
    }

    public String getMemoryReplacementStrategy(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.MEMORY_REPLACEMENT_STRATEGY_NAME).get();
    }

    public boolean getUseReplay(){
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.USE_REPLAY_NAME).get();
    }

    public boolean getUseLocalAdaptation(){
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.USE_LOCAL_ADAPTATION_NAME).get();
    }

    public int getLocalAdaptationK(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.LOCAL_ADAPTATION_K_NAME).get();
    }

    public int getlocalAdaptationGradientSteps(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.LOCAL_ADAPTATION_GRADIENT_STEPS_NAME).get();
    }

    public String getQueryNetDir(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.QUERY_NET_DIR_NAME).get();
    }
    
    public String getQueryNetPrefix(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.QUERY_NET_PREFIX_NAME).get();
    }

    public int getQueryNetNumInputs(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.QUERY_NET_NUM_INPUTS_NAME).get();
    }

    public int getValuesDim(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.VALUES_DIM_NAME).get();
    }

    public int getNodes(){
        return getLayerSymbol().getIntValue(AllPredefinedLayers.NODES_NAME).get();
    }

    @Nullable
    public List<Integer> getPadding(){

        if (getLayerSymbol().getIntTupleValue(AllPredefinedLayers.PADDING_NAME).isPresent()){
            return Arrays.asList(getLayerSymbol().getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(0),
                                getLayerSymbol().getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(1),
                                getLayerSymbol().getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(2));
        } else {

            String pad = ((LayerSymbol) getElement()).getStringValue(AllPredefinedLayers.PADDING_NAME).get();

            if(pad.equals("same")){
                return getPadding(getLayerSymbol()); //The padding calculated here is only used in the gluon/ mxnet backend, in the tensorlflow one it is interpreted as "same"
            }else if(pad.equals("valid")){
                return Arrays.asList(0,-1,0,0,0,0,0,0);
            }else if(pad.equals("same3d")){ //Three Dimensional Cases
                return getPadding3D(getLayerSymbol());
            }else if(pad.equals("valid3d")){
                return Arrays.asList(0,0,0);
            }else if(pad.equals("simple3d")){
                return Arrays.asList(1,1,1);
            }else{ //"no loss"
                return Arrays.asList(0,0,-1,0,0,0,0,0);
            }
        }
    }

    @Nullable
    public List<Integer> getPadding(LayerSymbol layer){
        List<Integer> kernel = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
        List<Integer> stride = layer.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);
        ArchTypeSymbol outputType = layer.getOutputTypes().get(0);

        int heightWithPad = kernel.get(0) + stride.get(0)*(outputType.getHeight() - 1);
        int widthWithPad = kernel.get(1) + stride.get(1)*(outputType.getWidth() - 1);
        int heightPad = Math.max(0, heightWithPad - inputType.getHeight());
        int widthPad = Math.max(0, widthWithPad - inputType.getWidth());

        int topPad = (int)Math.ceil(heightPad / 2.0);
        int bottomPad = (int)Math.floor(heightPad / 2.0);
        int leftPad = (int)Math.ceil(widthPad / 2.0);
        int rightPad = (int)Math.floor(widthPad / 2.0);

        if (topPad == 0 && bottomPad == 0 && leftPad == 0 && rightPad == 0){
            return null;
        }

        return Arrays.asList(0,0,0,0,topPad,bottomPad,leftPad,rightPad);
    }

    //NEW
    @Nullable
    public List<Integer> getPadding3D(LayerSymbol layer){
        List<Integer> kernel = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
        List<Integer> stride = layer.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);
        ArchTypeSymbol outputType = layer.getOutputTypes().get(0);

        int heightWithPad = kernel.get(1) + stride.get(1)*(outputType.getHeight() - 1);
        int widthWithPad = kernel.get(2) + stride.get(2)*(outputType.getWidth() - 1);
        int depthWithPad = kernel.get(0) + stride.get(0)*(outputType.getDepth() - 1);
        int heightPad = Math.max(0, heightWithPad - inputType.getHeight());
        int widthPad = Math.max(0, widthWithPad - inputType.getWidth());
        int depthPad = Math.max(0, depthWithPad - inputType.getDepth());

        int topPad = (int)Math.ceil(heightPad / 2.0);
        int bottomPad = (int)Math.floor(heightPad / 2.0);
        int leftPad = (int)Math.ceil(widthPad / 2.0);
        int rightPad = (int)Math.floor(widthPad / 2.0);
        int frontPad = (int)Math.ceil(depthPad / 2.0);
        int backPad = (int)Math.floor(depthPad / 2.0);

        if (topPad == 0 && bottomPad == 0 && leftPad == 0 && rightPad == 0 && frontPad == 0 && backPad == 0){
            return null;
        }

        return Arrays.asList(frontPad,topPad,leftPad);
    }

    @Nullable
    public List<Integer> getTransPadding3D(LayerSymbol layer) {
        List<Integer> kernel = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
        List<Integer> stride = layer.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);
        ArchTypeSymbol outputType = layer.getOutputTypes().get(0);

        int heightPad = (stride.get(1) - 1)*inputType.getHeight() - stride.get(1) + kernel.get(1);
        int widthPad = (stride.get(2) - 1) *inputType.getHeight() - stride.get(1) +  kernel.get(2);
        int depthPad = (stride.get(0) - 1)*inputType.getHeight() - stride.get(1) +  kernel.get(0);

        int topPad = (int) Math.ceil(heightPad / 2.0);
        int bottomPad = (int) Math.floor(heightPad / 2.0);
        int leftPad = (int) Math.ceil(widthPad / 2.0);
        int rightPad = (int) Math.floor(widthPad / 2.0);
        int frontPad = (int)Math.ceil(depthPad / 2.0);
        int backPad = (int)Math.floor(depthPad / 2.0);
        
        Integer toPrint = new Integer(heightPad);
        System.out.println(toPrint.toString());
        /*if (topPad == 0 && bottomPad == 0 && leftPad == 0 && rightPad == 0){
            return null;
        }*/
        //return Arrays.asList(0, 0, 0);
        return Arrays.asList(frontPad, topPad, leftPad);
    }

    @Nullable
    public List<Integer> getTransPadding(){
        if (getLayerSymbol().getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).isPresent()){
            return Arrays.asList(getLayerSymbol().getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(0),
                                getLayerSymbol().getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(1),
                                getLayerSymbol().getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(2));
        } else {

            String pad = ((LayerSymbol) getElement()).getStringValue(AllPredefinedLayers.TRANSPADDING_NAME).get();

            if(pad.equals("simple3d")){
                return Arrays.asList(1,1,1); //The padding calculated here is only used in the gluon/ mxnet backend, in the tensorlflow one it is interpreted as "same"
            } else if(pad.equals("same3d")){
                return getTransPadding3D(getLayerSymbol());
            } else if(pad.equals("valid3d")){
                return Arrays.asList(0,0,0);
            } else if(pad.equals("same")){
                return getTransPadding(getLayerSymbol());
            } else {
                return Arrays.asList(0,0);
            }
        }
    }

    @Nullable
    public List<Integer> getTransPadding(LayerSymbol layer) {
        List<Integer> kernel = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
        List<Integer> stride = layer.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
        ArchTypeSymbol inputType = layer.getInputTypes().get(0);
        ArchTypeSymbol outputType = layer.getOutputTypes().get(0);

        int heightPad = kernel.get(0) - stride.get(0);
        int widthPad = kernel.get(1) - stride.get(1);

        int topPad = (int) Math.ceil(heightPad / 2.0);
        int bottomPad = (int) Math.floor(heightPad / 2.0);
        int leftPad = (int) Math.ceil(widthPad / 2.0);
        int rightPad = (int) Math.floor(widthPad / 2.0);

        /*if (topPad == 0 && bottomPad == 0 && leftPad == 0 && rightPad == 0){
            return null;
        }*/

        return Arrays.asList(bottomPad, rightPad);
    }

    /*public boolean getStart() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.START_NAME).get();
    }

    public boolean getEnd() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.END_NAME).get();
    }*/
}
