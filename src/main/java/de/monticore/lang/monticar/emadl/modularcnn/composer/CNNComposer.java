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
import de.monticore.lang.monticar.emadl.modularcnn.builder.SymbolCreator;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

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

    private void writeNetworkFile(ASTComponent component){
        if (component == null) return;
        ComponentInformation componentInformation = new ComponentInformation(component, archNodes);
        try {
            writeToFile(componentInformation.getComponentName());
        } catch (Exception e) {
            e.printStackTrace();
        }


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

        writeNetworkFile(node.getComponent());

        //transformComponentToCNN(node);



        //TODO: Add to archNodes for completeness/updates
        //this.archNodes.add(new ArchitectureNode())

    }

    private void writeToFile(String content) throws IOException {
        String fileName = "ComposedNetworks.txt";
        File f = new File(fileName);

        if (!f.exists()){
            f.createNewFile();
        }

        BufferedWriter writer = new BufferedWriter(new FileWriter(fileName,true));
        writer.append(content + "\n");
        writer.close();

    }

    //TODO: Transform Node to architecture Node
    private void transformComponentToCNN(ASTEMACompilationUnit node){
        Log.info("" + node.toString(),"COMPOSED_CNN_TRANSFORM_START");
        ASTComponent component = node.getComponent();

        if (component.getSymbolOpt().isPresent()) {
            EMAComponentSymbol symbol = (EMAComponentSymbol) component.getSymbolOpt().get();

            //InstanceBuilder builder = new InstanceBuilder();
            //EMAComponentInstanceSymbol architectureSymbol2 = builder.build();

            SymbolCreator symbolCreator = new SymbolCreator();
            //symbolCreator.createAndLinkNewSymbol(component,symbol);
            symbolCreator.createInstances(symbol, symbol.getName());


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
        Log.info("" + node.toString(),"COMPOSED_CNN_TRANSFORM_END");

    }
    
    
    
}
