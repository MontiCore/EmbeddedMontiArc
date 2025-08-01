/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.compositions;


import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;


import java.util.ArrayList;



public class ArchitectureNodeScanner {
    ArrayList<ArchitectureNode> architecturesNodes = null;

    public ArchitectureNodeScanner(ArrayList<ArchitectureNode> currentNodes) {
        this.architecturesNodes = currentNodes;
    }



    public void scanForArchitectureNodes(ASTNode node){
        if (node instanceof ASTArchitecture){
           ArchitectureNode newArchNode = new ArchitectureNode((ASTArchitecture) node);

           for (ArchitectureNode architectureNode: architecturesNodes) {
                if (architectureNode.getComponentName() == newArchNode.getComponentName()) return;
           }

           architecturesNodes.add(newArchNode);
        }
    }
}
