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


import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.*;
import java.util.function.Function;

public class UnrollInstructionSymbol extends NetworkInstructionSymbol {

    public static final UnrollInstructionKind KIND = new UnrollInstructionKind();
    public static final Integer CONST_OFFSET = IODeclarationSymbol.MAX_ARRAY_LENGTH * 100;

    private UnrollDeclarationSymbol declaration = null;
    private List<ArgumentSymbol> arguments;
    private ParameterSymbol timeParameter;

    private ArrayList<SerialCompositeElementSymbol> bodies = new ArrayList<>();

    protected UnrollInstructionSymbol(String name) {
        super(name, KIND);
    }

    @Override
    public boolean isUnroll() {
        return true;
    }

    public UnrollDeclarationSymbol getDeclaration() {
        if (declaration == null) {
            Collection<UnrollDeclarationSymbol> collection = getEnclosingScope().resolveMany(getName(), UnrollDeclarationSymbol.KIND);

            if (!collection.isEmpty()) {
                declaration = collection.iterator().next();
            }
            else {
                throw new IllegalStateException("No unroll declaration found");
            }
        }
        return declaration;
    }
    public ArrayList<SerialCompositeElementSymbol> getBodiesForAllTimesteps() {
        return bodies;
    }

    @Override
    public boolean isResolvable() {
        return getBody().isResolvable() && getDeclaration() != null;
    }

    public List<ArgumentSymbol> getArguments() {
        return arguments;
    }

    protected void setArguments(List<ArgumentSymbol> arguments) {
        this.arguments = arguments;
    }

    public ParameterSymbol getTimeParameter(){
        return timeParameter;
    }

    protected void setTimeParameter(ParameterSymbol timeParameter){
        this.timeParameter = timeParameter;
        this.timeParameter.putInScope(getSpannedScope());
    }

    protected void putInScope(Scope scope){
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)){
            scope.getAsMutableScope().add(this);
            for (ArgumentSymbol argument : getArguments()){
                argument.putInScope(getSpannedScope().getAsMutableScope());
            }
        }
    }

    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                getDeclaration();
                resolveExpressions();

                int startValue = getTimeParameter().getDefaultExpression().get().getIntValue().get();
                int endValue = getIntValue(AllPredefinedLayers.MAX_LENGTH_NAME).get();

                getTimeParameter().getExpression().setValue(CONST_OFFSET);
                getBody().resolveOrError();

                for (int timestep = startValue; timestep < endValue; timestep++) {
                    SerialCompositeElementSymbol currentBody = getBody().preResolveDeepCopy();
                    currentBody.putInScope(getBody().getSpannedScope());

                    getTimeParameter().getExpression().setValue(timestep);
                    getTimeParameter().putInScope(currentBody.getEnclosingScope());

                    currentBody.resolveOrError();

                    bodies.add(currentBody);
                }


                UnrollInstructionSymbol resolvedUnroll = getDeclaration().call(this);
                setResolvedThis(resolvedUnroll);
            }
        }

        return getUnresolvableParameters();
    }

    protected void resolveExpressions() throws ArchResolveException{
        for (ArgumentSymbol argument : getArguments()){
            argument.resolveExpression();
        }
    }

    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        for (ArgumentSymbol argument : getArguments()){
            argument.getRhs().checkIfResolvable(allParameters);
            unresolvableParameters.addAll(argument.getRhs().getUnresolvableParameters());
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

    public void setIntValue(String parameterName, int value) {
        setTValue(parameterName, value, ArchSimpleExpressionSymbol::of);
    }

    public void setIntTupleValue(String parameterName, List<Object> tupleValues) {
        setTValue(parameterName, tupleValues, ArchSimpleExpressionSymbol::of);
    }

    public void setBooleanValue(String parameterName, boolean value) {
        setTValue(parameterName, value, ArchSimpleExpressionSymbol::of);
    }

    public void setStringValue(String parameterName, String value) {
        setTValue(parameterName, value, ArchSimpleExpressionSymbol::of);
    }

    public void setDoubleValue(String parameterName, double value) {
        setTValue(parameterName, value, ArchSimpleExpressionSymbol::of);
    }

    public void setValue(String parameterName, Object value) {
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(value);
        setTValue(parameterName, res, Function.identity());
    }

    public <T> void setTValue(String parameterName, T value, Function<T, ArchSimpleExpressionSymbol> of) {
        Optional<ParameterSymbol> param = getDeclaration().getParameter(parameterName);

        if (param.isPresent()) {
            Optional<ArgumentSymbol> arg = getArgument(parameterName);
            ArchSimpleExpressionSymbol expression = of.apply(value);

            if (arg.isPresent()) {
                arg.get().setRhs(expression);
            }
            else {
                arg = Optional.of(new ArgumentSymbol(parameterName));
                arg.get().setRhs(expression);
                arguments.add(arg.get());
            }
        }
    }

    protected ResolvableSymbol preResolveDeepCopy() {
        UnrollInstructionSymbol copy = new UnrollInstructionSymbol(getName());
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        List<ArgumentSymbol> args = new ArrayList<>(getArguments().size());
        for (ArgumentSymbol argument : getArguments()){
            args.add(argument.preResolveDeepCopy());
        }
        copy.setArguments(args);

        copy.setTimeParameter(getTimeParameter());
        copy.getTimeParameter().putInScope(copy.getSpannedScope());

        copy.setBody(getBody().preResolveDeepCopy());
        copy.getBody().putInScope(copy.getSpannedScope());

        return copy;
    }
}
