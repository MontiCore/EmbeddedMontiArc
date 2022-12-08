/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.compositions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.monticar.emadl.modularcnn.tools.ComposedNetworkFileHandler;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class CNNProcessor {
    private ArrayList<ArchitectureNode> archNodes = null;
    private String composedNetworksFilePath = "";
    public CNNProcessor(ArrayList<ArchitectureNode> currentNodes, String composedNetworksFilePath) {
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
    }
}
