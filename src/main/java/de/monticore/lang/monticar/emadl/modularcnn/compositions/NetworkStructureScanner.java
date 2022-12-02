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



public class NetworkStructureScanner{
    ArrayList<ArchitectureNode> architecturesNodes = null;

    public NetworkStructureScanner(ArrayList<ArchitectureNode> currentNodes) {
        this.architecturesNodes = currentNodes;
    }



    public void scanForArchitectureNodes(ASTNode node){
        if (node instanceof ASTArchitecture){
           for (ArchitectureNode architectureNode: architecturesNodes){
               if (!architectureNode.isComposedNode() && architectureNode.getOriginalNode().equals((ASTArchitecture) node)) return;
           }

           ArchitectureNode newArchNode = new ArchitectureNode((ASTArchitecture) node);

           for (ArchitectureNode architectureNode: architecturesNodes) {
                if (architectureNode.getComponentName() == newArchNode.getComponentName()) return;
           }

           architecturesNodes.add(newArchNode);
        }
    }
}
