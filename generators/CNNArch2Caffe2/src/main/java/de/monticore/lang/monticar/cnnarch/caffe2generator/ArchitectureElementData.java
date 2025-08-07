/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
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

    public List<String> getInputs(){
        return getTemplateController().getLayerInputs(getElement());
    }

    public boolean isLogisticRegressionOutput(){
        return getTemplateController().isLogisticRegressionOutput(getElement());
    }


    public boolean isLinearRegressionOutput(){
        boolean result = getTemplateController().isLinearRegressionOutput(getElement());
        if (result){
            Log.warn("The Output '" + getElement().getName() + "' is a linear regression output (squared loss) during training" +
                            " because the previous architecture element is not a softmax (cross-entropy loss) or sigmoid (logistic regression loss) activation. " +
                            "Other loss functions are currently not supported. "
                    , getElement().getSourcePosition());
        }
        return result;
    }

    public boolean isSoftmaxOutput(){
        return getTemplateController().isSoftmaxOutput(getElement());
    }




    public List<Integer> getKernel(){
        return ((LayerSymbol) getElement())
                .getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get();
    }

    public int getChannels(){
        return ((LayerSymbol) getElement())
                .getIntValue(AllPredefinedLayers.CHANNELS_NAME).get();
    }

    public List<Integer> getStride(){
        return ((LayerSymbol) getElement())
                .getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get();
    }

    public int getUnits(){
        return ((LayerSymbol) getElement())
                .getIntValue(AllPredefinedLayers.UNITS_NAME).get();
    }

    public boolean getNoBias(){
        return ((LayerSymbol) getElement())
                .getBooleanValue(AllPredefinedLayers.NOBIAS_NAME).get();
    }

    public double getP(){
        return ((LayerSymbol) getElement())
                .getDoubleValue(AllPredefinedLayers.P_NAME).get();
    }

    public int getIndex(){
        return ((LayerSymbol) getElement())
                .getIntValue(AllPredefinedLayers.INDEX_NAME).get();
    }

    public int getNumOutputs(){
        return ((LayerSymbol) getElement())
                .getIntValue(AllPredefinedLayers.NUM_SPLITS_NAME).get();
    }

    public boolean getFixGamma(){
        return ((LayerSymbol) getElement())
                .getBooleanValue(AllPredefinedLayers.FIX_GAMMA_NAME).get();
    }

    public int getNsize(){
        return ((LayerSymbol) getElement())
                .getIntValue(AllPredefinedLayers.NSIZE_NAME).get();
    }

    public double getKnorm(){
        return ((LayerSymbol) getElement())
                .getDoubleValue(AllPredefinedLayers.KNORM_NAME).get();
    }

    public double getAlpha(){
        return ((LayerSymbol) getElement())
                .getDoubleValue(AllPredefinedLayers.ALPHA_NAME).get();
    }

    public double getBeta(){
        return ((LayerSymbol) getElement())
                .getDoubleValue(AllPredefinedLayers.BETA_NAME).get();
    }

    @Nullable
    public String getPoolType(){
        return ((LayerSymbol) getElement())
                .getStringValue(AllPredefinedLayers.POOL_TYPE_NAME).get();
    }

    @Nullable
    public Integer getPadding(){
        return getPadding((LayerSymbol) getElement());
    }

    @Nullable
    public Integer getPadding(LayerSymbol layer){
        String padding_type = ((LayerSymbol) getElement()).getStringValue(AllPredefinedLayers.PADDING_NAME).get();
        Integer pad=0;

        if (padding_type.equals(AllPredefinedLayers.PADDING_VALID)){
            pad = 0;
        } else if (padding_type.equals(AllPredefinedLayers.PADDING_SAME)){
            pad = 1;
        }

        return pad;
    }
}
