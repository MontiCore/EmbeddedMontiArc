/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

abstract public class BaseRNN extends PredefinedLayerDeclaration {

    public BaseRNN(String name) {
        super(name);
    }

    @Override
    public boolean isTrainable(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE;
    }

    @Override
    public int getArrayLength(VariableSymbol.Member member) {
        if (member == VariableSymbol.Member.NONE ||
            member == VariableSymbol.Member.STATE ||
            member == VariableSymbol.Member.OUTPUT) {
            return 1;
        }

        return 0;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        boolean bidirectional = layer.getBooleanValue(AllPredefinedLayers.BIDIRECTIONAL_NAME).get();
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();

        if (member == VariableSymbol.Member.STATE) {
            int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(bidirectional ? 2 * layers : layers)
                    .height(units)
                    .width(1)
                    .elementType("-oo", "oo")
                    .build());
        }
        else {
            return Collections.singletonList(new ArchTypeSymbol.Builder()
                    .channels(layer.getInputTypes().get(0).getChannels())
                    .height(bidirectional ? 2 * units : units)
                    .width(1)
                    .elementType("-oo", "oo")
                    .build());
        }
    }

    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        boolean bidirectional = layer.getBooleanValue(AllPredefinedLayers.BIDIRECTIONAL_NAME).get();
        int units = layer.getIntValue(AllPredefinedLayers.UNITS_NAME).get();
        int layers = layer.getIntValue(AllPredefinedLayers.LAYERS_NAME).get();

        errorIfInputSizeIsNotOne(inputTypes, layer);
        errorIfInputWidthIsInvalid(inputTypes, layer, 1);

        if (member == VariableSymbol.Member.STATE) {
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, bidirectional ? 2 * layers : layers);
            errorIfInputHeightIsInvalid(inputTypes, layer, units);
        }
        else {
            errorIfInputChannelSizeIsInvalid(inputTypes, layer, layer.getInputTypes().get(0).getChannels());
        }
    }

    @Override
    public boolean canBeInput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.OUTPUT || member == VariableSymbol.Member.STATE;
    }

    @Override
    public boolean canBeOutput(VariableSymbol.Member member) {
        return member == VariableSymbol.Member.NONE || member == VariableSymbol.Member.STATE;
    }

    protected static List<ParameterSymbol> createParameters() {
        return new ArrayList<>(Arrays.asList(
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.UNITS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.LAYERS_NAME)
                        .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                        .defaultValue(1)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.RNN_DROPOUT_NAME)
                        .constraints(Constraints.NUMBER, Constraints.BETWEEN_ZERO_AND_ONE)
                        .defaultValue(0)
                        .build(),
                new ParameterSymbol.Builder()
                        .name(AllPredefinedLayers.BIDIRECTIONAL_NAME)
                        .constraints(Constraints.BOOLEAN)
                        .defaultValue(false)
                        .build()));
    }
}
