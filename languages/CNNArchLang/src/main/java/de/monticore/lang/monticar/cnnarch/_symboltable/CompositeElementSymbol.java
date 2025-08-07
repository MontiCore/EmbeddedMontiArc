/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.*;

public abstract class CompositeElementSymbol extends ArchitectureElementSymbol {

    protected List<ArchitectureElementSymbol> elements = new ArrayList<>();


    public CompositeElementSymbol() {
        super("");
        setResolvedThis(this);
    }

    abstract protected void setElements(List<ArchitectureElementSymbol> elements);

    public List<ArchitectureElementSymbol> getElements() {
        return elements;
    }

    @Override
    public boolean isAtomic() {
        return getElements().isEmpty();
    }

    @Override
    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                resolveExpressions();

                for (ArchitectureElementSymbol element : getElements()) {
                    element.resolve();
                }
            }
        }
        return getUnresolvableParameters();
    }

    @Override
    public boolean containsAdaNet() {
        return super.containsAdaNet();
    }
    public void setAdaNet(boolean adaNet){super.setAdaNet(adaNet);}
    @Override
    protected void resolveExpressions() throws ArchResolveException {
        for (ArchitectureElementSymbol element : getElements()){
            element.resolveExpressions();
        }
    }

    @Override
    public boolean isResolved() {
        boolean isResolved = true;
        for (ArchitectureElementSymbol element : getElements()){
            if (!element.isResolved()){
                isResolved = false;
            }
        }
        return isResolved;
    }

    @Override
    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        for (ArchitectureElementSymbol element : getElements()){
            element.checkIfResolvable(allParameters);
            unresolvableParameters.addAll(element.getUnresolvableParameters());
        }
    }

    @Override
    protected void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);
            for (ArchitectureElementSymbol element : getElements()) {
                element.putInScope(getSpannedScope());
            }
        }
    }
}
