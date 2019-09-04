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

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class ConstantSymbol extends ArchitectureElementSymbol {

    private ArchSimpleExpressionSymbol expression = null;

    protected ConstantSymbol() {
        super("const");
    }

    public ArchSimpleExpressionSymbol getExpression() {
        return expression;
    }

    protected void setExpression(ArchSimpleExpressionSymbol expression) {
        this.expression = expression;
    }

    @Override
    public boolean isResolvable() {
        return super.isResolvable();
    }

    @Override
    public boolean isAtomic() {
        return getResolvedThis().isPresent() && getResolvedThis().get() == this;
    }

    @Override
    public List<ArchitectureElementSymbol> getFirstAtomicElements() {
        if (getResolvedThis().isPresent() && getResolvedThis().get() != this) {
            return ((ArchitectureElementSymbol) getResolvedThis().get()).getFirstAtomicElements();
        }
        else {
            return Collections.singletonList(this);
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getLastAtomicElements() {
        if (getResolvedThis().isPresent() && getResolvedThis().get() != this) {
            return ((ArchitectureElementSymbol) getResolvedThis().get()).getLastAtomicElements();
        }
        else {
            return Collections.singletonList(this);
        }
    }

    @Override
    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                resolveExpressions();
                setResolvedThis(this);
            }
        }
        return getUnresolvableParameters();
    }

    @Override
    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        getExpression().checkIfResolvable(allParameters);
        unresolvableParameters.addAll(getExpression().getUnresolvableParameters());
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes() {
        List<ArchTypeSymbol> outputShapes;

        if (isAtomic()) {
            ArchTypeSymbol outputShape = new ArchTypeSymbol();

            // Since symbol is resolved at this point, it is safe to assume that the expression is an int
            int value = getExpression().getIntValue().get();

            ASTRange range = new ASTRange();
            range.setStartValue(String.valueOf(value));
            range.setEndValue(String.valueOf(value));
            ASTElementType domain = new ASTElementType("Z", Optional.of(range));
            outputShape.setDomain(domain);

            outputShapes = Collections.singletonList(outputShape);
        }
        else {
            if (!getResolvedThis().isPresent()){
                throw new IllegalStateException("The architecture resolve() method was never called");
            }
            outputShapes = ((ArchitectureElementSymbol) getResolvedThis().get()).computeOutputTypes();
        }

        return outputShapes;
    }

    @Override
    public void checkInput() {
        if (isAtomic()) {
            if (!getInputTypes().isEmpty()) {
                Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid number of input streams. "
                          , getSourcePosition());
            }
        }
        else {
            if (!getResolvedThis().isPresent()) {
                throw new IllegalStateException("The architecture resolve() method was never called");
            }
            ((ArchitectureElementSymbol) getResolvedThis().get()).checkInput();
        }
    }

    @Override
    public Optional<Integer> getParallelLength() {
        return Optional.of(1);
    }

    @Override
    public Optional<List<Integer>> getSerialLengths() {
        return Optional.of(Collections.nCopies(getParallelLength().get(), 1));
    }

    @Override
    protected void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);

            getExpression().putInScope(getSpannedScope());
        }
    }

    @Override
    protected void resolveExpressions() throws ArchResolveException {
        getExpression().resolveOrError();

        if (!Constraints.INTEGER.check(getExpression(), getSourcePosition(), getName())) {
            throw new ArchResolveException();
        }
    }

    @Override
    protected ArchitectureElementSymbol preResolveDeepCopy() {
        ConstantSymbol copy = new ConstantSymbol();

        if (getAstNode().isPresent()) {
            copy.setAstNode(getAstNode().get());
        }

        copy.setExpression(getExpression().preResolveDeepCopy());

        return copy;
    }
}
