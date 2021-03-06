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
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
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
                    Log.warn("The input resolution is smaller than the kernel. " +
                                    "This results in an output resolution of 1x1. " +
                                    "If this warning appears multiple times, consider changing your architecture"
                            , layer.getSourcePosition());
                }
            }
        }
    }


    //output type function for convolution and pooling
    protected static List<ArchTypeSymbol> computeConvAndPoolOutputShape(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        String borderModeSetting = method.getStringValue(AllPredefinedLayers.PADDING_NAME).get();
        if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME)) {
            return computeOutputShapeWithSamePadding(inputType, method, channels);
        } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID)) {
            return computeOutputShapeWithValidPadding(inputType, method, channels);
        } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_NO_LOSS)) {
            return computeOutputShapeWithNoLossPadding(inputType, method, channels);
        } else {
            throw new IllegalStateException("border_mode is " + borderModeSetting + ". This should never happen.");
        }
    }

    //output type function for transposed convolution
    protected static List<ArchTypeSymbol> computeUpConvOutputShape(ArchTypeSymbol inputType, LayerSymbol method, int channels) {
        String borderModeSetting = method.getStringValue(AllPredefinedLayers.TRANSPADDING_NAME).get();
        if (borderModeSetting.equals(AllPredefinedLayers.PADDING_SAME)) {
            return computeUpConvOutputShapeWithSamePadding(inputType, method, channels);
        } else if (borderModeSetting.equals(AllPredefinedLayers.PADDING_VALID)) {
            return computeUpConvOutputShapeWithValidPadding(inputType, method, channels);
        } else {
            throw new IllegalStateException("border_mode is " + borderModeSetting + ". This should never happen.");
        }
    }

    protected static void computeOneHotOutputSize(LayerSymbol layer) {
        int outputChannels = 0;

        if (layer.getOutputElement().get() instanceof VariableSymbol && layer.getOutputElement().get().isOutput()) {
            outputChannels = ((VariableSymbol) layer.getOutputElement().get()).getIoDeclaration().getType().getChannels();
        }
        layer.setIntValue(AllPredefinedLayers.SIZE_NAME, outputChannels);
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
