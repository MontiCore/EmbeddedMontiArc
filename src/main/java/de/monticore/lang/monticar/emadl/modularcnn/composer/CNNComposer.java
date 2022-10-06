/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl.modularcnn.tools.ComposedNetworkFileHandler;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class CNNComposer {
    private ArrayList<ArchitectureNode> archNodes = null;
    private String composedNetworksFilePath = "";
    public CNNComposer(ArrayList<ArchitectureNode> currentNodes, String composedNetworksFilePath) {
        this.archNodes = currentNodes;
        this.composedNetworksFilePath = composedNetworksFilePath;
    }

    private boolean isValidCNNCanditate(ASTComponent component){
        if (component == null) return false;

        ComponentInformation componentInformation = new ComponentInformation(component, archNodes);
        Log.info(component.getName() + " is composed CNN: " + componentInformation.isComposedCNN(),"IS_COMPOSED_CNN");
        return componentInformation.isComposedCNN();
    }

    public boolean checkNotNullAndValid(ASTEMACompilationUnit node){
        ASTComponent component = node.getComponent();
        return isValidCNNCanditate(component);
    }

    public void checkAndProcessComponentOnMatch(ASTEMACompilationUnit node) {
        if (!checkNotNullAndValid(node)) return;
        ComponentInformation componentInformation = new ComponentInformation(node.getComponent(), archNodes);

        ComposedNetworkFileHandler composedNetworkFileHandler = new ComposedNetworkFileHandler(this.composedNetworksFilePath);
        composedNetworkFileHandler.documentNetworkInFile(componentInformation);

        transformComponentToCNN(node);

        //TODO: Add to archNodes for completeness/updates
        //this.archNodes.add(new ArchitectureNode())
    }

    //TODO: Transform Node to architecture Node

    private void transformComponentToCNN(ASTEMACompilationUnit node){
        Log.info("" + node.toString(),"COMPOSED_CNN_TRANSFORM_START");
        ASTComponent component = node.getComponent();


        if (component.getSymbolOpt().isPresent()) {
            EMAComponentSymbol symbol = (EMAComponentSymbol) component.getSymbolOpt().get();

            //InstanceBuilder builder = new InstanceBuilder();
            //EMAComponentInstanceSymbol architectureSymbol2 = builder.build();

            /*SymbolCreator symbolCreator = new SymbolCreator();
            //symbolCreator.createAndLinkNewSymbol(component,symbol);
            symbolCreator.createInstances(symbol, symbol.getName());
            */

            ArchitectureSymbol architectureSymbol = new ArchitectureSymbol();
            Optional<ArchitectureSymbol> architectureSymbolOpt = Optional.of(architectureSymbol);
            component.setSymbol(architectureSymbol);
            component.setSymbolOpt(architectureSymbolOpt);

            //InstanceBuilder instanceBuilder = new InstanceBuilder(node);
            /*
            ModifiedExpandedInstanceSymbolCreator instanceSymbolCreator = new ModifiedExpandedInstanceSymbolCreator();
            String instanceName = symbol.getName();
            instanceSymbolCreator.createInstances(symbol,instanceName);
            instanceSymbolCreator.createInstances((EMAComponentSymbol)(Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null))),node.getComponent().getSymbolOpt().get().getName());
            */
      }
        //Log.info("" + node.toString(),"COMPOSED_CNN_TRANSFORM_END");
    }
}
