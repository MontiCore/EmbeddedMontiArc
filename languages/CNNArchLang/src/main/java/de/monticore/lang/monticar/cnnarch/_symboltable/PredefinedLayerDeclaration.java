/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;
import java.util.function.BinaryOperator;
import java.util.stream.Stream;

abstract public class PredefinedLayerDeclaration extends LayerDeclarationSymbol implements LayerComputeOutputTypes{

    public PredefinedLayerDeclaration(String name) {
        super(name);
    }

    @Override
    protected void setParameters(List<ParameterSymbol> parameters) {
        super.setParameters(parameters);
        for (ParameterSymbol param : parameters) {
            param.putInScope(getSpannedScope());
        }
    }

    @Override
    public boolean isPredefined() {
        return true;
    }

    public boolean isTrainable(VariableSymbol.Member member) {
        if (member == VariableSymbol.Member.STATE || member == VariableSymbol.Member.OUTPUT) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * A note to this function: For occurrences without the use of VariableSymbol these are the same but when using the
     * layer as a layer variable and accessing its member OUT, you generally want to use the LayerSymbol's input types
     * since they have the input types of the VariableSymbol occurrence with member == NONE. It makes sense as the actual
     * layer call determines the types, not the use of the (already generated) output of the layer.
     * tl;dr: Always use LayerSymbol's input types (layer.getInputTypes()) unless you know what you do.
     */


    abstract public void checkInput(List<ArchTypeSymbol> inputTypes,
                                    LayerSymbol layer,
                                    VariableSymbol.Member member);

    public int getArrayLength(VariableSymbol.Member member) {
        if (member == VariableSymbol.Member.NONE || member == VariableSymbol.Member.OUTPUT) {
            return 1;
        }

        return 0;
    }

    public boolean canBeInput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.OUTPUT;
    }

    public boolean canBeOutput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE;
    }

    @Override
    public PredefinedLayerDeclaration deepCopy() {
        throw new IllegalStateException("Copy method should not be called for predefined layer declarations.");
    }

    //the following methods are only here to avoid duplication. They are used by multiple subclasses.

    //check if inputTypes is of size 1
    protected void errorIfInputSizeIsNotOne(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        if (inputTypes.size() != 1) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                            getName() + " layer can only handle one input stream. " +
                            "Current number of input streams " + inputTypes.size() + "."
                    , layer.getSourcePosition());
        }
    }

    protected void errorIfInputIsEmpty(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        if (inputTypes.size() == 0) {
            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Number of input streams is 0"
                    , layer.getSourcePosition());
        }
    }

    protected void errorIfInputChannelSizeIsInvalid(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, int channels) {
        for (ArchTypeSymbol inputType : inputTypes) {
            if (inputType.getChannels() != channels) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Input channel size is "
                                + inputType.getChannels() + " but needs to be " + channels + "."
                        , layer.getSourcePosition());
            }
        }
    }

    protected void errorIfInputHeightIsInvalid(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, int height) {
        for (ArchTypeSymbol inputType : inputTypes) {
            if (inputType.getHeight() != height) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Input height is "
                                + inputType.getHeight() + " but needs to be " + height + "."
                        , layer.getSourcePosition());
            }
        }
    }

    protected void errorIfInputWidthIsInvalid(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, int width) {
        for (ArchTypeSymbol inputType : inputTypes) {
            if (inputType.getWidth() != width) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Input width is "
                                + inputType.getWidth() + " but needs to be " + width + "."
                        , layer.getSourcePosition());
            }
        }
    }

    protected void errorIfInputDepthIsInvalid(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, int depth){
        for (ArchTypeSymbol inputType : inputTypes) {
            if (inputType.getDepth() != depth){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. Input depth is "
                                + inputType.getDepth() + " but needs to be " + depth + ".", layer.getSourcePosition());
            }
        }
    }

    protected static void errorIfInputSmallerThanKernel3D(List<ArchTypeSymbol> inputTypes, LayerSymbol layer){
        if (!inputTypes.isEmpty()) {
            int inputHeight = inputTypes.get(0).getHeight();
            int inputWidth = inputTypes.get(0).getWidth();
            int inputDepth = inputTypes.get(0).getDepth();
            Integer depthI = new Integer(inputTypes.get(0).getDepthIndex());

            int kernelHeight = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
            int kernelWidth = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
            int kernelDepth = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

            if (kernelHeight > inputHeight || kernelWidth > inputWidth || kernelDepth > inputDepth) {
                if (layer.getStringValue(AllPredefinedLayers.PADDING_NAME).equals(AllPredefinedLayers.PADDING_VALID3D)) {
                    Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                    "The input resolution is smaller than the kernel and the padding mode is 'valid'." +
                                    "This would result in an output resolution of 0x0."
                            , layer.getSourcePosition());
                } else {
                    Integer height = new Integer(kernelHeight);
                    Integer width = new Integer(kernelWidth);
                    Integer depth = new Integer(kernelDepth);
                    Integer iheight = new Integer(inputHeight);
                    Integer iwidth = new Integer(inputWidth);
                    Integer idepth = new Integer(inputDepth);
                    Integer channel = new Integer(inputTypes.get(0).getChannels());
                    Log.warn("The input resolution is smaller than the kernel. " + " " +
                                    "This results in an output resolution of 1x1. " + " " +
                                    "If this warning appears multiple times, consider changing your architecture"
                            , layer.getSourcePosition());
                }
            } 
        }
    }

    protected static enum HandlingSingleInputs {
        ALLOWED, IGNORED, RESTRICTED
    }

    /**
     * Check if Inputs of Layer have the same shape
     * @param inputTypes: List of input Types
     * @param layer: curremt Layer
     * @param handling: HandlingSingleInputs Enum Value, either ALLOWED, IGNORED or RESTRICTED
     * ALLOWED will skip the Check if there are only one Input.
     * IGNORED will print a message that the Layer is redundant, but the Input may still be passed
     * RESTRICTED will throw an error. This indicates that the Layer does not allow
     */
    protected static void errorIfMultipleInputShapesAreNotEqual(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, HandlingSingleInputs handling) {
        if (inputTypes.size() == 1){
            if (handling == HandlingSingleInputs.IGNORED) {
                Log.warn(layer.getName() + " layer has only one input stream. Layer can be removed.", layer.getSourcePosition());
            } else if (handling == HandlingSingleInputs.RESTRICTED){
                Log.error(layer.getName() + " layer has only one input stream.", layer.getSourcePosition());
            }
        }
        else if (inputTypes.size() > 1){
            List<Integer> heightList = new ArrayList<>();
            List<Integer> widthList = new ArrayList<>();
            List<Integer> channelsList = new ArrayList<>();
            for (ArchTypeSymbol shape : inputTypes){
                heightList.add(shape.getHeight());
                widthList.add(shape.getWidth());
                channelsList.add(shape.getChannels());
            }
            int countEqualHeights = (int)heightList.stream().distinct().count();
            int countEqualWidths = (int)widthList.stream().distinct().count();
            int countEqualNumberOfChannels = (int)channelsList.stream().distinct().count();
            if (countEqualHeights != 1 || countEqualWidths != 1 || countEqualNumberOfChannels != 1){
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                "Shapes of all input streams must be equal. " +
                                "Input heights: " + Joiners.COMMA.join(heightList) + ". " +
                                "Input widths: " + Joiners.COMMA.join(widthList) + ". " +
                                "Number of input channels: " + Joiners.COMMA.join(channelsList) + ". "
                        , layer.getSourcePosition());
            }
        }
    }

    protected static void errorIfInputNotFlattened(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        if (!inputTypes.isEmpty()) {
            for (ArchTypeSymbol inputType : layer.getInputTypes()) {
                int height = inputType.getHeight();
                int width = inputType.getWidth();
                if (height != 1 || width != 1) {
                    Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input." +
                            " Input layer must be flat, consider using a 'Flatten()' layer.", layer.getSourcePosition());
                }
            }
        }
    }

    //check input for convolution and pooling
    protected static void errorIfInputSmallerThanKernel(List<ArchTypeSymbol> inputTypes, LayerSymbol layer) {
        if (!inputTypes.isEmpty()) {
            int inputHeight = inputTypes.get(0).getHeight();
            int inputWidth = inputTypes.get(0).getWidth();

            int kernelHeight = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);
            int kernelWidth = layer.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);

            if (kernelHeight > inputHeight || kernelWidth > inputWidth) {
                if (layer.getStringValue(AllPredefinedLayers.PADDING_NAME).equals(AllPredefinedLayers.PADDING_VALID)) {
                    Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. " +
                                    "The input resolution is smaller than the kernel and the padding mode is 'valid'." +
                                    "This would result in an output resolution of 0x0."
                            , layer.getSourcePosition());
                } else {
                    Integer height = new Integer(kernelHeight);
                    Integer width = new Integer(kernelWidth);
                    Integer iheight = new Integer(inputHeight);
                    Integer iwidth = new Integer(inputWidth);
                    Log.warn("The input resolution is smaller than the kernel. " + " " +
                                    "This results in an output resolution of 1x1. " +
                                    "If this warning appears multiple times, consider changing your architecture"
                            , layer.getSourcePosition());
                }
            } 
        }
    }

    //output type function for convolution and poolingee
    protected static List<ArchTypeSymbol> computeConvAndPoolOutputShape(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        if (method.getIntTupleValue(AllPredefinedLayers.PADDING_NAME).isPresent()){ //If the Padding is given in Tuple
            return computeOutputShapeWithTupelPadding3D(inputType, method, channels);
        } else {
            String borderModeSetting = method.getStringValue(AllPredefinedLayers.PADDING_NAME).get();
            if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME)) {
                return computeOutputShapeWithSamePadding(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID)) {
                return computeOutputShapeWithValidPadding(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_NO_LOSS)) {
                return computeOutputShapeWithNoLossPadding(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME3D)){ //Add 3D PADDING CASES
                return computeOutputShapeWithSamePadding3D(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID3D)){
                return computeOutputShapeWithValidPadding3D (inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SIMPLE3D)){
                return computeOutputShapeWithSimplePadding3D (inputType, method, channels);
            } else {
                throw new IllegalStateException("border_mode is " + borderModeSetting + ". This should never happen.");
            }
        }
    }

    //output type function for transposed convolution
    protected static List<ArchTypeSymbol> computeUpConvOutputShape(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        if (method.getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).isPresent()){ //If the Padding is given in Tuple
            return computeUpOutputShapeWithTupelPadding3D(inputType, method, channels);
        } else {
            String borderModeSetting = method.getStringValue(AllPredefinedLayers.TRANSPADDING_NAME).get();
            if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME)) {
                return computeUpConvOutputShapeWithSamePadding(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID)) {
                return computeUpConvOutputShapeWithValidPadding(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME3D)){ //Add 3D PADDING CASES
                return computeUpConvOutputShapeWithSamePadding3D(inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID3D)){
                return computeUpConvOutputShapeWithValidPadding3D (inputType, method, channels);
            } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SIMPLE3D)){
                return computeUpConvOutputShapeWithSimplePadding3D (inputType, method, channels);
            } else {
                throw new IllegalStateException("border_mode is " + borderModeSetting + ". This should never happen.");
            }
        }
    }

    protected static void computeOneHotOutputSize(LayerSymbol layer) {
        int outputChannels = 0;

        if (layer.getOutputElement().get() instanceof VariableSymbol && layer.getOutputElement().get().isOutput()) {
            outputChannels = ((VariableSymbol) layer.getOutputElement().get()).getIoDeclaration().getType().getChannels();
        }
        layer.setIntValue(AllPredefinedLayers.SIZE_NAME, outputChannels);
    }

    //Same padding should result in the output shape having the same size as the input shape
    private static List<ArchTypeSymbol> computeOutputShapeWithSamePadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth = (inputWidth + strideWidth - 1) / strideWidth;
        int outputHeight = (inputHeight + strideHeight - 1) / strideHeight;
        int outputDepth = (inputDepth + strideDepth - 1) / strideDepth;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //Padding of size (0,0,0)
    private static List<ArchTypeSymbol> computeOutputShapeWithValidPadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth;
        int outputHeight;
        int outputDepth;
        if (inputWidth < kernelWidth || inputHeight < kernelHeight || inputDepth < kernelDepth) {
            outputWidth = 0;
            outputHeight = 0;
            outputDepth = 0;
        } else {
            outputWidth = 1 + ((inputWidth - kernelWidth) / strideWidth);
            outputHeight = 1 + ((inputHeight - kernelHeight) / strideHeight);
            outputDepth = 1 + ((inputDepth - kernelDepth) / strideDepth);
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //Padding of size (1,1,1)
    private static List<ArchTypeSymbol> computeOutputShapeWithSimplePadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int pad = 1;

        int outputWidth;
        int outputHeight;
        int outputDepth;

        if (inputWidth + 2*pad < kernelWidth || inputHeight + 2*pad < kernelHeight || inputDepth + 2*pad < kernelDepth) {
            outputWidth = 0;
            outputHeight = 0;
            outputDepth = 0;
        } else {
            outputWidth = 1 + ((inputWidth - kernelWidth + 2* pad) / strideWidth);
            outputHeight = 1 + ((inputHeight - kernelHeight + 2* pad) / strideHeight);
            outputDepth = 1 + ((inputDepth - kernelDepth + 2* pad) / strideDepth);
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    private static List<ArchTypeSymbol> computeOutputShapeWithTupelPadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int paddingHeight = method.getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(1);
        int paddingWidth = method.getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(2);
        int paddingDepth = method.getIntTupleValue(AllPredefinedLayers.PADDING_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth = 1 + ((inputWidth - kernelWidth + 2*paddingWidth) / strideWidth);
        int outputHeight = 1 + ((inputHeight - kernelHeight + 2*paddingHeight) / strideHeight);
        int outputDepth = 1 + ((inputDepth - kernelDepth + 2*paddingDepth) / strideDepth);

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }


    private static List<ArchTypeSymbol> computeUpConvOutputShapeWithSamePadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth;
        int outputHeight;
        int outputDepth;
        
        outputWidth = strideWidth*inputWidth;
        outputHeight = strideHeight*inputHeight;
        outputDepth = strideDepth*inputDepth;

        //Integer oDepth = new Integer(outputDepth);
        //System.out.println("In computeUpConvShape: " + iDepth.toString() + " " + stride.toString() + " " + oDepth.toString());

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    private static List<ArchTypeSymbol> computeUpConvOutputShapeWithValidPadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth;
        int outputHeight;
        int outputDepth;

        //TEMPORARY
        int pad = 0;

        outputWidth = (inputWidth - 1) * strideWidth + kernelWidth - 2*pad;
        outputHeight = (inputHeight - 1) * strideHeight + kernelHeight - 2*pad;
        outputDepth = (inputDepth - 1) * strideDepth + kernelDepth - 2*pad;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    private static List<ArchTypeSymbol> computeUpConvOutputShapeWithSimplePadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth;
        int outputHeight;
        int outputDepth;

        int pad = 1;

        outputWidth = (inputWidth - 1) * strideWidth + kernelWidth - 2*pad;
        outputHeight = (inputHeight - 1) * strideHeight + kernelHeight - 2*pad;
        outputDepth = (inputDepth - 1) * strideDepth + kernelDepth - 2*pad;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    private static List<ArchTypeSymbol> computeUpOutputShapeWithTupelPadding3D(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(2);
        int strideDepth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(2);
        int kernelDepth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);

        int paddingHeight = method.getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(1);
        int paddingWidth = method.getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(2);
        int paddingDepth = method.getIntTupleValue(AllPredefinedLayers.TRANSPADDING_NAME).get().get(0);

        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();
        int inputDepth = inputType.getDepth();

        int outputWidth = (inputWidth - 1) * strideWidth + kernelWidth - 2*paddingWidth;
        int outputHeight = (inputHeight - 1) * strideHeight + kernelHeight - 2*paddingHeight;
        int outputDepth = (inputDepth - 1) * strideDepth + kernelDepth - 2*paddingDepth;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .depth(outputDepth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build()); 
    }


    //padding with border_mode=valid, no padding
    private static List<ArchTypeSymbol> computeOutputShapeWithValidPadding(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();

        int outputWidth;
        int outputHeight;
        if (inputWidth < kernelWidth || inputHeight < kernelHeight) {
            outputWidth = 0;
            outputHeight = 0;
        } else {
            outputWidth = 1 + (inputWidth - kernelWidth) / strideWidth;
            outputHeight = 1 + (inputHeight - kernelHeight) / strideHeight;
        }

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //padding with border_mode=valid, no padding
    private static List<ArchTypeSymbol> computeUpConvOutputShapeWithValidPadding(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();

        int outputWidth;
        int outputHeight;

        outputWidth = (inputWidth - 1) * strideWidth + kernelWidth;
        outputHeight = (inputHeight - 1) * strideHeight + kernelHeight;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //padding until no data gets discarded, same as valid with a stride of 1
    private static List<ArchTypeSymbol> computeOutputShapeWithNoLossPadding(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int kernelHeight = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(0);
        int kernelWidth = method.getIntTupleValue(AllPredefinedLayers.KERNEL_NAME).get().get(1);
        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();

        int outputWidth = 1 + Math.max(0, ((inputWidth - kernelWidth + strideWidth - 1) / strideWidth));
        int outputHeight = 1 + Math.max(0, ((inputHeight - kernelHeight + strideHeight - 1) / strideHeight));

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //padding with border_mode='same'
    private static List<ArchTypeSymbol> computeOutputShapeWithSamePadding(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();

        int outputWidth = (inputWidth + strideWidth - 1) / strideWidth;
        int outputHeight = (inputHeight + strideWidth - 1) / strideHeight;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

    //padding with border_mode='same'
    private static List<ArchTypeSymbol> computeUpConvOutputShapeWithSamePadding(ArchTypeSymbol inputType, LayerSymbol method, int channels){
        int strideHeight = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(0);
        int strideWidth = method.getIntTupleValue(AllPredefinedLayers.STRIDE_NAME).get().get(1);
        int inputHeight = inputType.getHeight();
        int inputWidth = inputType.getWidth();

        int outputWidth = inputWidth * strideWidth;
        int outputHeight = inputHeight * strideHeight;

        return Collections.singletonList(new ArchTypeSymbol.Builder()
                .height(outputHeight)
                .width(outputWidth)
                .channels(channels)
                .elementType("-oo", "oo")
                .build());
    }

        protected List<String> computeStartAndEndValue
        (List < ArchTypeSymbol > inputTypes, BinaryOperator < Rational > startValAccumulator, BinaryOperator < Rational > endValAccumulator)
        {
            Stream.Builder<Rational> startValues = Stream.builder();
            Stream.Builder<Rational> endValues = Stream.builder();
            String start = null;
            String end = null;
            for (ArchTypeSymbol inputType : inputTypes) {
                Optional<ASTRange> range = inputType.getDomain().getRangeOpt();
                if (range.isPresent()) {
                    if (range.get().hasNoLowerLimit()) {
                        start = "-oo";
                    } else {
                        startValues.add(range.get().getStartValue());
                    }
                    if (range.get().hasNoUpperLimit()) {
                        end = "oo";
                    } else {
                        endValues.add(range.get().getEndValue());
                    }
                }
            }
            if (start == null) {
                start = "" + startValues.build().reduce(startValAccumulator).get().doubleValue();
            }
            if (end == null) {
                end = "" + endValues.build().reduce(endValAccumulator).get().doubleValue();
            }

            return Arrays.asList(start, end);
        }
    }
