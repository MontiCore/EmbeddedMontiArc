/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.emadl.modularcnn.builder.InstanceBuilder;
import de.monticore.lang.monticar.emadl.modularcnn.builder.SymbolCreator;
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

    public boolean checkNotNullAndValid(ASTEMACompilationUnit node){
        ASTComponent component = node.getComponent();
        return component != null && isValidCNNCanditate(component);
    }


    public void checkAndTransformComponentOnMatch(ASTEMACompilationUnit node) {
        if (!checkNotNullAndValid(node)) return;
        transformComponentToCNN(node);

        //TODO: Add to archNodes for completeness/updates
        //this.archNodes.add(new ArchitectureNode())

    }

    //TODO: Transform Node to architecture Node
    private void transformComponentToCNN(ASTEMACompilationUnit node){
        if (node.getComponent().getSymbolOpt().isPresent()) {
            EMAComponentSymbol symbol = (EMAComponentSymbol) node.getComponent().getSymbolOpt().get();

            SymbolCreator symbolCreator = new SymbolCreator();
            symbolCreator.createInstances(symbol, symbol.getName());

            //InstanceBuilder instanceBuilder = new InstanceBuilder(node);
            /*
            ModifiedExpandedInstanceSymbolCreator instanceSymbolCreator = new ModifiedExpandedInstanceSymbolCreator();
            String instanceName = symbol.getName();
            instanceSymbolCreator.createInstances(symbol,instanceName);
            instanceSymbolCreator.createInstances((EMAComponentSymbol)(Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null))),node.getComponent().getSymbolOpt().get().getName());
            */
        }


    }
    
    
    
}
