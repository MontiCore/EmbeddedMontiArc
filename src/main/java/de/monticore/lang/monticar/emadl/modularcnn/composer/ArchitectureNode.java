/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.emadl.modularcnn.tools.ScopeFinder;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.Scope;

import java.util.ArrayList;

public class ArchitectureNode{

    final private ArrayList<ASTArchitecture> originalNodes;
    final private ScopeFinder scopeFinder = new ScopeFinder();

    private boolean composedNode = false;
    private String componentName;

    public ArchitectureNode(ArrayList<ASTArchitecture> nodes, String composedComponentName){
        this.originalNodes = nodes;
        this.composedNode = true;
        this.componentName = composedComponentName;
    }

    public ArchitectureNode(ASTArchitecture node){
        this.originalNodes = new ArrayList<>();
        this.originalNodes.add(node);
        this.composedNode = false;
        this.componentName = findComponentName(node);
    }

    public boolean isComposedNode(){
        return composedNode;
    }

    public ASTArchitecture getOriginalNode() {
        if (!composedNode && originalNodes.size() == 1) return this.originalNodes.get(0);
        return null;
    }

    public ArrayList<ASTArchitecture> getOriginalNodes(){
        if (this.originalNodes.size() > 0) return this.originalNodes;
        return null;
    }

    public String getComponentName() {
        return componentName;
    }

    private String findComponentName(ASTArchitecture node) {
        if (!node.getEnclosingScopeOpt().isPresent()) return null;

        Scope nodeScope = node.getEnclosingScopeOpt().get();
        ArtifactScope artifactScope = scopeFinder.getNextArtifactScopeUp(nodeScope);

        if (artifactScope.getName().isPresent()) return artifactScope.getName().get();
        return null;
    }
}
