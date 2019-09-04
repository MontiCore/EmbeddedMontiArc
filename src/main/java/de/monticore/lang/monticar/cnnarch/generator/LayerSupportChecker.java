package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ConstantSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public abstract class LayerSupportChecker {

    protected List<String> supportedLayerList = new ArrayList<>();

    private boolean isSupportedLayer(ArchitectureElementSymbol element){
        List<ArchitectureElementSymbol> constructLayerElemList;

        if (element instanceof CompositeElementSymbol) {
            constructLayerElemList = ((CompositeElementSymbol) element).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement)) {
                    return false;
                }
            }
            return true;
        }

        // Support all inputs and outputs
        if (element instanceof VariableSymbol) {
            if (((VariableSymbol) element).getType() == VariableSymbol.Type.LAYER) {
                return isSupportedLayer(((VariableSymbol) element).getLayerVariableDeclaration().getLayer());
            }
            else if (element.isInput() || element.isOutput()) {
                return true;
            }
        }

        // Support for constants is checked in ArchitectureSupportChecker
        if (element instanceof ConstantSymbol) {
            return true;
        }

        // Support all layer declarations
        if (element instanceof LayerSymbol) {
            if (!((LayerSymbol) element).getDeclaration().isPredefined()) {
                return true;
            }
        }

        if (!supportedLayerList.contains(element.toString())) {
            Log.error("Unsupported layer " + "'" + element.getName() + "'" + " for the current backend.");
            return false;
        } else {
            return true;
        }
    }

    public boolean check(ArchitectureSymbol architecture) {
        for (CompositeElementSymbol stream : architecture.getStreams()) {
            for (ArchitectureElementSymbol element : stream.getElements()) {
                if (!isSupportedLayer(element)) {
                    return false;
                }
            }
        }

        for (UnrollSymbol unroll: architecture.getUnrolls()) {
            for (ArchitectureElementSymbol element : unroll.getBody().getElements()) {
                if (!isSupportedLayer(element)) {
                    return false;
                }
            }
        }

        return true;
    }

}
