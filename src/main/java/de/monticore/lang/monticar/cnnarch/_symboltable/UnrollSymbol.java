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
import java.util.function.Function;

public class UnrollSymbol extends ArchitectureElementSymbol {

    protected List<ArchitectureElementSymbol> elements = new ArrayList<>();
    private UnrollDeclarationSymbol declaration = null;
    private List<ArgumentSymbol> arguments;

    public UnrollSymbol(String name) {
        super(name);
        setResolvedThis(this);
    }

    public List<ArchitectureElementSymbol> getElements() {
        return elements;
    }

    public boolean isTrainable() {
        boolean isTrainable = false;

        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof CompositeElementSymbol) {
                isTrainable |= ((CompositeElementSymbol) element).isTrainable();
            }
            else if (element instanceof LayerSymbol) {
                isTrainable |= ((LayerSymbol) element).getDeclaration().isTrainable();
            }
        }

        return isTrainable;
    }

    @Override
    public boolean isResolvable() {
        return super.isResolvable() && getDeclaration() != null;
    }


    public UnrollDeclarationSymbol getDeclaration() {
        if (declaration == null){
            Collection<UnrollDeclarationSymbol> declarationCollection = getEnclosingScope().resolveMany(getName(), UnrollDeclarationSymbol.KIND);
            if (!declarationCollection.isEmpty()){
                setDeclaration(declarationCollection.iterator().next());
            }
        }
        return declaration;
    }

    private void setDeclaration(UnrollDeclarationSymbol declaration) {
        this.declaration = declaration;
    }

    public List<ArgumentSymbol> getArguments() {
        return arguments;
    }

    protected void setArguments(List<ArgumentSymbol> arguments) {
        this.arguments = arguments;
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

    protected void setElements(List<ArchitectureElementSymbol> elements) {
        ArchitectureElementSymbol previous = null;
        for (ArchitectureElementSymbol current : elements){
            if (previous != null){
                current.setInputElement(previous);
                previous.setOutputElement(current);
            }
            else {
                if (getInputElement().isPresent()){
                    current.setInputElement(getInputElement().get());
                }
                if (getOutputElement().isPresent()){
                    current.setOutputElement(getOutputElement().get());
                }
            }
            previous = current;
        }
        this.elements = elements;
    }

    @Override
    public void setInputElement(ArchitectureElementSymbol inputElement) {
        super.setInputElement(inputElement);
        if (!getElements().isEmpty()){
            getElements().get(0).setInputElement(inputElement);
        }
    }

    @Override
    public void setOutputElement(ArchitectureElementSymbol outputElement) {
        super.setOutputElement(outputElement);
        if (!getElements().isEmpty()){
            getElements().get(getElements().size()-1).setOutputElement(outputElement);
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getFirstAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            return getElements().get(0).getFirstAtomicElements();
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getLastAtomicElements() {
        if (getElements().isEmpty()){
            return Collections.singletonList(this);
        }
        else {
            return getElements().get(getElements().size()-1).getLastAtomicElements();
        }
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes() {
        if (getElements().isEmpty()){
            if (getInputElement().isPresent()){
                return getInputElement().get().getOutputTypes();
            }
            else {
                return Collections.emptyList();
            }
        }
        else {
            for (ArchitectureElementSymbol element : getElements()){
                element.getOutputTypes();
            }
            return getElements().get(getElements().size() - 1).getOutputTypes();
        }
    }

    public Optional<ArgumentSymbol> getArgument(String name){
        for (ArgumentSymbol argument : getArguments()){
            if (argument.getName().equals(name)) {
                return Optional.of(argument);
            }
        }
        return Optional.empty();
    }

    public Optional<Integer> getIntValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getIntValue);
    }

    public Optional<List<Integer>> getIntTupleValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getIntTupleValues);
    }

    public Optional<Boolean> getBooleanValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getBooleanValue);
    }

    public Optional<String> getStringValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getStringValue);
    }

    public Optional<Double> getDoubleValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getDoubleValue);
    }

    public Optional<Object> getValue(String parameterName){
        return getTValue(parameterName, ArchExpressionSymbol::getValue);
    }

    public <T> Optional<T> getTValue(String parameterName, Function<ArchExpressionSymbol, Optional<T>> getValue){
        Optional<ArgumentSymbol> arg = getArgument(parameterName);
        Optional<ParameterSymbol> param = getDeclaration().getParameter(parameterName);
        if (arg.isPresent()){
            return getValue.apply(arg.get().getRhs());
        }
        else if (param.isPresent() && param.get().getDefaultExpression().isPresent()){
            return getValue.apply(param.get().getDefaultExpression().get());
        }
        else {
            return Optional.empty();
        }
    }

    @Override
    public void checkInput() {
        if (getResolvedThis().isPresent()){
            if (getResolvedThis().get() == this){
                //((PredefinedUnrollDeclaration) getDeclaration()).checkInput(getInputTypes(), this);
            }
            else {
                getResolvedThis().get().checkInput();
            }
        }
    }

    @Override
    public Optional<Integer> getParallelLength() {
        return Optional.of(1);
    }

    @Override
    public Optional<List<Integer>> getSerialLengths() {
        return Optional.of(Collections.singletonList(getElements().size()));
    }

    @Override
    protected SerialCompositeElementSymbol preResolveDeepCopy() {
        SerialCompositeElementSymbol copy = new SerialCompositeElementSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        List<ArchitectureElementSymbol> elements = new ArrayList<>(getElements().size());
        for (ArchitectureElementSymbol element : getElements()){
            ArchitectureElementSymbol elementCopy = element.preResolveDeepCopy();
            elements.add(elementCopy);
        }
        copy.setElements(elements);
        return copy;
    }

}
