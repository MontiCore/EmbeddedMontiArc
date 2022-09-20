/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

public class ScopeFinder {

    public ScopeFinder() {}

    public ArtifactScope getNextArtifactScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof ArtifactScope) return (ArtifactScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextArtifactScopeUp(scope.getEnclosingScope().get());
        return null;
    }

    public GlobalScope getNextGlobalScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof GlobalScope) return (GlobalScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextGlobalScopeUp(scope.getEnclosingScope().get());
        return null;
    }

    public CommonScope getNextCommonScopeUp(Scope scope){
        if (scope == null) return null;
        else if (scope instanceof CommonScope) return (CommonScope) scope;
        else if (scope.getEnclosingScope().isPresent()) return getNextCommonScopeUp(scope.getEnclosingScope().get());
        return null;
    }
}
