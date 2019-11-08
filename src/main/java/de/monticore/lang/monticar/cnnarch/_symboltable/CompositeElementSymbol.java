/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
