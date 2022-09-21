/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.emadl.modularcnn.ScopeFinder;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.Scope;

public class ArchitectureNode{

    private ASTArchitecture originalNode = null;
    private String componentName = null;
    public ScopeFinder scopeFinder = new ScopeFinder();

    public ArchitectureNode(ASTArchitecture node){
        this.originalNode = node;
        this.componentName = findComponentName(node);
    }

    public ASTArchitecture getOriginalNode() {
        return originalNode;
    }

    public String getComponentName() {
        return componentName;
    }

    private String findComponentName(ASTArchitecture node) /*throws Exception*/ {
        if (!node.getEnclosingScopeOpt().isPresent()) {
            //System.exit(0);
            //throw new Exception("Enclosing Scope is null");
            return null;
        }

        Scope nodeScope = node.getEnclosingScopeOpt().get();
        ArtifactScope artifactScope = scopeFinder.getNextArtifactScopeUp(nodeScope);

        if (artifactScope.getName().isPresent()) return artifactScope.getName().get();
        return null;
    }
}
