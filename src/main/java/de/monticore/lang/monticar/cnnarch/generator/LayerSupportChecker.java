package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public abstract class LayerSupportChecker {

    protected List<String> supportedLayerList = new ArrayList<>();

    private boolean isSupportedLayer(ArchitectureElementSymbol element){
        ArchitectureElementSymbol resolvedElement = element.getResolvedThis().get();
        List<ArchitectureElementSymbol> constructLayerElemList;

        if (resolvedElement instanceof CompositeElementSymbol) {
            constructLayerElemList = ((CompositeElementSymbol) resolvedElement).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement)) {
                    return false;
                }
            }
            return true;
        }

        // Support all inputs and outputs
        if (resolvedElement.isInput() || resolvedElement.isOutput()) {
            return true;
        }

        // Support for constants is checked in ArchitectureSupportChecker
        if (resolvedElement instanceof ConstantSymbol) {
            return true;
        }

        // Support all layer declarations
        if (resolvedElement instanceof LayerSymbol) {
            if (!((LayerSymbol) resolvedElement).getDeclaration().isPredefined()) {
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
