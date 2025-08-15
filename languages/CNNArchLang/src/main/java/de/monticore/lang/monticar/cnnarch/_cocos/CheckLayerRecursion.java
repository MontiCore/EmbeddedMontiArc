/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class CheckLayerRecursion implements CNNArchASTLayerDeclarationCoCo {

    Set<LayerDeclarationSymbol> seenLayers = new HashSet<>();
    boolean done;

    @Override
    public void check(ASTLayerDeclaration node) {
        done = false;
        LayerDeclarationSymbol layerDeclaration = (LayerDeclarationSymbol) node.getSymbolOpt().get();
        checkForRecursion(layerDeclaration, layerDeclaration.getBody());
    }

    private void checkForRecursion(LayerDeclarationSymbol startingLayer, ArchitectureElementSymbol current){
        if (!done) {
            if (current instanceof CompositeElementSymbol) {
                for (ArchitectureElementSymbol architectureElement : ((CompositeElementSymbol) current).getElements()) {
                    checkForRecursion(startingLayer, architectureElement);
                }
            }
            else if (current instanceof LayerSymbol) {
                LayerDeclarationSymbol layerDeclaration = ((LayerSymbol) current).getDeclaration();
                if (layerDeclaration != null && !layerDeclaration.isPredefined() && !seenLayers.contains(layerDeclaration)) {
                    seenLayers.add(layerDeclaration);
                    if (startingLayer == layerDeclaration) {
                        Log.error("0" + ErrorCodes.RECURSION_ERROR + " Recursion is not allowed. " +
                                        "The layer '" + startingLayer.getName() + "' creates a recursive cycle."
                                , startingLayer.getSourcePosition());
                        done = true;
                    }
                    else {
                        checkForRecursion(startingLayer, layerDeclaration.getBody());
                    }
                }
            }
        }
    }

}
