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

    public void checkAndProcessComponentOnMatch(ASTEMACompilationUnit node) {
        if (node.getComponent() == null) return;
        ComponentInformation componentInformation = new ComponentInformation(node.getComponent(), archNodes);

        if (componentInformation.isComposedCNN()){
            ComposedNetworkFileHandler composedNetworkFileHandler = new ComposedNetworkFileHandler(this.composedNetworksFilePath);
            composedNetworkFileHandler.documentNetworkInFile(componentInformation);
        }
    }
}
