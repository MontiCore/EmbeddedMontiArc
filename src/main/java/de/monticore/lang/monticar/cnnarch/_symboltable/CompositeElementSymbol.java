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

    public List<ArchitectureElementSymbol> getElements() {
        return elements;
    }

    abstract protected void setElements(List<ArchitectureElementSymbol> elements);

    public boolean isTrainable() {
        boolean isTrainable = false;

        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof CompositeElementSymbol) {
                isTrainable |= ((CompositeElementSymbol) element).isTrainable();
            }
            else if (element instanceof LayerSymbol) {
                isTrainable |= ((LayerSymbol) element).getDeclaration().isTrainable();
            }
            else if (element instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) element;

                if (variable.getType() == VariableSymbol.Type.LAYER) {
                    LayerDeclarationSymbol layerDeclaration = ((LayerVariableDeclarationSymbol) variable.getDeclaration()).getLayer().getDeclaration();

                    if (layerDeclaration.isPredefined()) {
                        isTrainable |= ((PredefinedLayerDeclaration) layerDeclaration).isTrainable(variable.getMember());
                    }
                    else {
                        isTrainable |= layerDeclaration.isTrainable();
                    }
                }
            }
        }

        return isTrainable;
    }

    @Override
    public boolean isAtomic() {
        return getElements().isEmpty();
    }

    @Override
    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                List<ArchitectureElementSymbol> resolvedElements = new ArrayList<>();
                for (ArchitectureElementSymbol element : getElements()) {
                    element.resolve();
                }
            }
        }
        return getUnresolvableParameters();
    }

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
