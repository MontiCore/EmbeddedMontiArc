/* (c) https://github.com/MontiCore/monticore */
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
        ArchitectureElementSymbol resolvedElement = (ArchitectureElementSymbol) element.getResolvedThis().get();

        if (resolvedElement instanceof CompositeElementSymbol) {
            List<ArchitectureElementSymbol> constructLayerElemList = ((CompositeElementSymbol) resolvedElement).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement)) {
                    return false;
                }
            }
            return true;
        }

        // Support all inputs and outputs
        if (resolvedElement instanceof VariableSymbol) {
            if (((VariableSymbol) resolvedElement).getType() == VariableSymbol.Type.LAYER) {
                return isSupportedLayer(((VariableSymbol) resolvedElement).getLayerVariableDeclaration().getLayer());
            }
            else if (resolvedElement.isInput() || resolvedElement.isOutput()) {
                return true;
            }
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
        for (NetworkInstructionSymbol networkInstructions : architecture.getNetworkInstructions()) {
            for (ArchitectureElementSymbol element : networkInstructions.getBody().getElements()) {
                if (!isSupportedLayer(element)) {
                    return false;
                }
            }
        }

        return true;
    }

}
