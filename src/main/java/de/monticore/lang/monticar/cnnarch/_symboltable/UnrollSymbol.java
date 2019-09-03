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
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.*;
import java.util.function.Function;

public class UnrollSymbol extends CommonScopeSpanningSymbol {

    public static final UnrollKind KIND = new UnrollKind();

    private UnrollDeclarationSymbol declaration = null;
    private List<ArgumentSymbol> arguments;
    private ParameterSymbol timeParameter;
    private UnrollSymbol resolvedThis = null;
    private SerialCompositeElementSymbol body;
    private ArrayList<SerialCompositeElementSymbol> bodies = new ArrayList<>();
    private boolean isExtendedForBackend = false;

    public SerialCompositeElementSymbol getBody() {
        return body;
    }

    protected void setBody(SerialCompositeElementSymbol body) {
        this.body = body;
    }

    public ArrayList<SerialCompositeElementSymbol> getBodiesForAllTimesteps() {
        return bodies;
    }

    protected void setBodiesForAllTimesteps(ArrayList<SerialCompositeElementSymbol> bodies) {
        this.bodies = bodies;
    }

    public boolean isExtended(){
        return this.isExtendedForBackend;
    }

    private void setExtended(boolean extended){
        this.isExtendedForBackend = extended;
    }

    public boolean isTrainable() {
        return body.isTrainable();
    }

    protected UnrollSymbol(String name) {
        super(name, KIND);
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


    public boolean isResolvable() {
        return getBody().isResolvable() && getDeclaration() != null;
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

    public ParameterSymbol getTimeParameter(){
        return timeParameter;
    }

    protected void setTimeParameter(ParameterSymbol timeParameter){
        this.timeParameter = timeParameter;
    }

    public ArchExpressionSymbol getIfExpression(){
        Optional<ArgumentSymbol> argument = getArgument(AllPredefinedVariables.CONDITIONAL_ARG_NAME);
        if (argument.isPresent()){
            return argument.get().getRhs();
        }
        else {
            return ArchSimpleExpressionSymbol.of(true);
        }
    }

    protected void putInScope(Scope scope){
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)){
            scope.getAsMutableScope().add(this);
            /*if (getResolvedThis().isPresent()){
                getResolvedThis().get().putInScope(getSpannedScope());
            }*/
            for (ArgumentSymbol argument : getArguments()){
                argument.putInScope(getSpannedScope().getAsMutableScope());
            }
        }
    }

    protected void setResolvedThis(UnrollSymbol resolvedThis) {
        if (resolvedThis != null){
            //resolvedThis.putInScope(getSpannedScope());
        }
        this.resolvedThis = resolvedThis;
    }



    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (true) {
            if (isResolvable()) {
                getDeclaration();
                resolveExpressions();

                //resolve the unroll call
                getBody().resolveOrError();
                UnrollSymbol resolvedUnroll = getDeclaration().call(this);
                setResolvedThis(resolvedUnroll);
            }
        }
        return new HashSet<ParameterSymbol>() ;
    }

    private boolean isActive(){
        if (getIfExpression().isSimpleValue() && !getIfExpression().getBooleanValue().get()){
            return false;
        }
        else {
            return true;
        }
    }

    protected void resolveExpressions() throws ArchResolveException{
        for (ArgumentSymbol argument : getArguments()){
            argument.resolveUnrollExpression();
        }
    }


    private ArchitectureElementSymbol createSerialSequencePart(List<ArchitectureElementSymbol> elements){
        if (elements.size() == 1){
            return elements.get(0);
        }
        else {
            SerialCompositeElementSymbol serialComposite = new SerialCompositeElementSymbol();
            serialComposite.setElements(elements);

            if (getAstNode().isPresent()){
                serialComposite.setAstNode(getAstNode().get());
            }
            return serialComposite;
        }
    }




    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableVariables, Set<ParameterSymbol> allVariables) {
        for (ArgumentSymbol argument : getArguments()){
            argument.getRhs().checkIfResolvable(allVariables);
            unresolvableVariables.addAll(argument.getRhs().getUnresolvableParameters());
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

    // creates a body for each timestep of the unroll
    public UnrollSymbol createUnrollForBackend(){
        if(this.isExtendedForBackend){
            return this;
        }else {
            int timestep;
            SerialCompositeElementSymbol newBody;
            ArchitectureSymbol architecture = getBody().getArchitecture();
            List<ArchitectureElementSymbol> newBodyList;

            for (timestep = this.getIntValue(AllPredefinedLayers.T_NAME).get(); timestep < this.getIntValue(AllPredefinedLayers.MAX_LENGTH_NAME).get(); timestep++) {
                newBody = new SerialCompositeElementSymbol();
                newBodyList = new ArrayList<>();
                SerialCompositeElementSymbol body = this.getBody().preResolveDeepCopy();
                body.putInScope(this.getBody().getSpannedScope());

                for (ArchitectureElementSymbol element : body.getElements()) {

                    if(element.getEnclosingScope() == null) {
                        element.setEnclosingScope(getEnclosingScope().getAsMutableScope());
                    }

                    ((ParameterSymbol)((ArrayList<Symbol>)getSpannedScope().getLocalSymbols().get(AllPredefinedLayers.T_NAME)).get(0)).getExpression().setValue(timestep);

                    try {
                        this.resolveExpressions();

                        for(ParameterSymbol p:declaration.getParameters()){
                            if(p.getEnclosingScope() == null) {
                                p.putInScope(getSpannedScope());
                            }
                        }

                        element.resolve();
                    } catch (ArchResolveException e) {e.printStackTrace();}

                    newBodyList.add(element);
                }
                newBody.putInScope(this.getBody().getSpannedScope());
                newBody.setElements(newBodyList);
                bodies.add(newBody);
            }
            newBody = new SerialCompositeElementSymbol();
            ArrayList elementsList = new ArrayList();
            elementsList.addAll(bodies);

            this.setBody(newBody);
            this.setExtended(true);

            List<UnrollSymbol> newUnrolls = new ArrayList<>();
            newUnrolls.add(this);
            architecture.setUnrolls(newUnrolls);
            return this;
        }
    }


    protected UnrollSymbol preResolveDeepCopy() {
        UnrollSymbol copy = new UnrollSymbol(getName());
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        List<ArgumentSymbol> args = new ArrayList<>(getArguments().size());
        for (ArgumentSymbol argument : getArguments()){
            args.add(argument.preResolveDeepCopy());
        }

        copy.setDeclaration(getDeclaration());

        List<ParameterSymbol> parameterCopies = new ArrayList<>(getDeclaration().getParameters().size());
        for (ParameterSymbol parameter : getDeclaration().getParameters()){
            ParameterSymbol parameterCopy = parameter.deepCopy();
            parameterCopies.add(parameterCopy);
            parameterCopy.putInScope(copy.getSpannedScope());
        }
        copy.getDeclaration().setParameters(parameterCopies);
        copy.setArguments(args);

        copy.setTimeParameter(getTimeParameter().deepCopy());
        // TODO: currently only the defaultValue for t is used, make it use the assigned value instead
        ((ParameterSymbol)((ArrayList<Symbol>)getSpannedScope().getLocalSymbols().get("t")).get(0)).getExpression().setValue(this.getIntValue(AllPredefinedLayers.T_NAME));
        copy.getTimeParameter().putInScope(getSpannedScope());


        copy.setBody(getBody().preResolveDeepCopy());
        copy.getBody().putInScope(copy.getSpannedScope());
        //TODO: Find a nicer way to put the timeParameter into the unroll elements' scope
        for(ArchitectureElementSymbol element: copy.getBody().getElements()){
            element.getSpannedScope().add(copy.getTimeParameter());
        }

        return copy;
    }

    public static class Builder{
        private UnrollDeclarationSymbol declaration;
        private List<ArgumentSymbol> arguments = new ArrayList<>();
        private boolean isResolved = false;

        public Builder declaration(UnrollDeclarationSymbol declaration){
            this.declaration = declaration;
            return this;
        }

        public Builder arguments(List<ArgumentSymbol> arguments){
            this.arguments = arguments;
            return this;
        }

        public Builder arguments(ArgumentSymbol... arguments){
            this.arguments = Arrays.asList(arguments);
            return this;
        }

        public Builder isResolved(boolean isResolved){
            this.isResolved = isResolved;
            return this;
        }



    }

}
