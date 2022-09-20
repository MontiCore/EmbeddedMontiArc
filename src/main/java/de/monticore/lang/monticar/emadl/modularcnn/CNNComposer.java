/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl._symboltable.ModifiedExpandedInstanceSymbolCreator;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class CNNComposer {

    private ArrayList<ArchitectureNode> archNodes = null;
    public CNNComposer(ArrayList<ArchitectureNode> currentNodes) {
        this.archNodes = currentNodes;
    }
    
    private boolean isValidCNNCanditate(ASTComponent component){
        if (component == null) return false;

        ComponentInformation componentInformation = new ComponentInformation(component, archNodes);
        Log.info(component.getName() + " is composed CNN: " + componentInformation.isComposedCNN(),"COMPOSED_CNN");
        return componentInformation.isComposedCNN();
    }

    public boolean nodeIsComposedCNN(ASTEMACompilationUnit node){
        if (node == null) return false;
        return isValidCNNCanditate(node.getComponent());
    }

    public void checkAndTransformComponentOnMatch(ASTEMACompilationUnit node) {
        ASTComponent component = node.getComponent();
        if (component == null || !isValidCNNCanditate(component)) return;

        transformComponentToCNN(node);

        //TODO: Add to archNodes for completeness/updates
        //this.archNodes.add(new ArchitectureNode())

    }

    //TODO: Transform Node to architecture Node
    private void transformComponentToCNN(ASTEMACompilationUnit node){
        ModifiedExpandedInstanceSymbolCreator instanceSymbolCreator = new ModifiedExpandedInstanceSymbolCreator();
        instanceSymbolCreator.createInstances((EMAComponentSymbol)(Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null))),node.getComponent().getSymbolOpt().get().getName());



    }
    
    
    
}
