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
import java.util.Arrays;
import java.util.List;

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

    public boolean getFlatten() {
        return getLayerSymbol().getBooleanValue(AllPredefinedLayers.FLATTEN_PARAMETER_NAME).get();
    }

    @Nullable
    public String getPoolType(){
        return getLayerSymbol().getStringValue(AllPredefinedLayers.POOL_TYPE_NAME).get();
    }

    @Nullable
    public List<Integer> getPadding(){

    	String pad = ((LayerSymbol) getElement()).getStringValue(AllPredefinedLayers.PADDING_NAME).get();

        if(pad.equals("same")){
            return getPadding(getLayerSymbol()); //The padding calculated here is only used in the gluon/ mxnet backend, in the tensorlflow one it is interpreted as "same"
        }else if(pad.equals("valid")){
            return Arrays.asList(0,-1,0,0,0,0,0,0);
        }else{ //"no loss"
            return Arrays.asList(0,0,-1,0,0,0,0,0);
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

    @Nullable
    public List<Integer> getTransPadding(){

        String pad = ((LayerSymbol) getElement()).getStringValue(AllPredefinedLayers.TRANSPADDING_NAME).get();

        if(pad.equals("same")){
            return getTransPadding(getLayerSymbol()); //The padding calculated here is only used in the gluon/ mxnet backend, in the tensorlflow one it is interpreted as "same"
        }else if(pad.equals("valid")){
            return Arrays.asList(0,0);
        }else{ //"no loss"
            return Arrays.asList(0,0,-1,0,0,0,0,0);
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