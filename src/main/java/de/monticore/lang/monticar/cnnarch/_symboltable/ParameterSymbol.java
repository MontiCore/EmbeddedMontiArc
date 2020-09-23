/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

import static de.monticore.lang.monticar.cnnarch.helper.ErrorCodes.MISSING_VAR_VALUE;

public class ParameterSymbol extends CommonSymbol {

    public static final ParameterKind KIND = new ParameterKind();

    private ParameterType type;
    private ArchSimpleExpressionSymbol defaultExpression = null; //Optional
    private ArchSimpleExpressionSymbol currentExpression = null; //Optional
    private Set<Constraints> constraints = new HashSet<>();

    protected ParameterSymbol(String name) {
        super(name, KIND);
    }

    public ParameterType getType() {
        return type;
    }

    protected void setType(ParameterType type) {
        this.type = type;
    }

    public Optional<ArchSimpleExpressionSymbol> getDefaultExpression() {
        return Optional.ofNullable(defaultExpression);
    }

    protected void setDefaultExpression(ArchSimpleExpressionSymbol defaultExpression) {
        if (this.defaultExpression != null && (getType() == ParameterType.CONSTANT)){
            throw new IllegalStateException("Invalid constant assignment.");
        }
        else {
            this.defaultExpression = defaultExpression;
        }
    }

    protected Optional<ArchSimpleExpressionSymbol> getCurrentExpression() {
        return Optional.ofNullable(currentExpression);
    }

    public Set<Constraints> getConstraints() {
        return constraints;
    }

    protected void setConstraints(Set<Constraints> constraints) {
        this.constraints = constraints;
    }

    public void addConstraint(Constraints... constraints) {
        for (int i = 0; i < constraints.length; i++){
            getConstraints().add(constraints[i]);
        }
    }

    public boolean isArchitectureParameter(){
        return type == ParameterType.ARCHITECTURE_PARAMETER;
    }

    public boolean isConstant(){
        return type == ParameterType.CONSTANT;
    }

    public boolean isLayerParameter(){
        return type == ParameterType.LAYER_PARAMETER;
    }

    public boolean isTimeParameter(){
        return type == ParameterType.TIME_PARAMETER;
    }

    public boolean hasExpression(){
        return getCurrentExpression().isPresent() || getDefaultExpression().isPresent();
    }

    public void setExpression(ArchSimpleExpressionSymbol expression){
        if (getType() != ParameterType.CONSTANT){
            currentExpression = expression;
        }
        else {
            throw new IllegalStateException("assigned new value to a constant");
        }
    }

    public ArchSimpleExpressionSymbol getExpression(){
        ArchSimpleExpressionSymbol value = null;
        if (hasExpression()){
            if (getCurrentExpression().isPresent()){
                value = getCurrentExpression().get();
            }
            else {
                value = getDefaultExpression().get();
            }
        }
        else {
            String msg = "0" + MISSING_VAR_VALUE + " Missing value for parameter  " + getName() + ".";
            if (getAstNode().isPresent()){
                Log.error(msg, getAstNode().get().get_SourcePositionStart());
            }
            else {
                Log.error(msg);
            }
        }
        return value;
    }

    public Optional<Object> getValue(){
        return getExpression().getValue();
    }

    public void reset(){
        currentExpression = null;
    }

    public void putInScope(Scope scope){
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);
            if (getDefaultExpression().isPresent()){
                getDefaultExpression().get().putInScope(scope);
            }
        }
    }

    public ParameterSymbol deepCopy() {
        ParameterSymbol copy = new ParameterSymbol(getName());
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        copy.setType(getType());
        copy.setConstraints(getConstraints());
        if (getDefaultExpression().isPresent()){
            copy.setDefaultExpression(getDefaultExpression().get().preResolveDeepCopy());
        }
        return copy;
    }


    public static class Builder{
        private ParameterType type = ParameterType.LAYER_PARAMETER;
        private ArchSimpleExpressionSymbol defaultValue = null;
        private String name = null;
        private Set<Constraints> constraints = new HashSet<>();

        public Builder type(ParameterType type){
            this.type = type;
            return this;
        }

        public Builder name(String name){
            this.name = name;
            return this;
        }

        public Builder defaultValue(ArchSimpleExpressionSymbol defaultValue){
            this.defaultValue = defaultValue;
            return this;
        }

        public Builder defaultValue(int defaultValue){
            this.defaultValue = ArchSimpleExpressionSymbol.of(defaultValue);
            return this;
        }

        public Builder defaultValue(double defaultValue){
            this.defaultValue = ArchSimpleExpressionSymbol.of(defaultValue);
            return this;
        }

        public Builder defaultValue(boolean defaultValue){
            this.defaultValue = ArchSimpleExpressionSymbol.of(defaultValue);
            return this;
        }

        public Builder defaultValue(String defaultValue){
            this.defaultValue = ArchSimpleExpressionSymbol.of(defaultValue);
            return this;
        }

        public Builder defaultValue(List<Object> tupleValues){
            this.defaultValue = ArchSimpleExpressionSymbol.of(tupleValues);
            return this;
        }

        public Builder constraints(Constraints... constraints){
            this.constraints = new HashSet<>(Arrays.asList(constraints));
            return this;
        }

        public ParameterSymbol build(){
            if (name == null || name.equals("")){
                throw new IllegalStateException("Missing or empty name for ParameterSymbol");
            }
            ParameterSymbol sym = new ParameterSymbol(name);
            sym.setType(type);
            sym.setDefaultExpression(defaultValue);
            sym.setConstraints(constraints);
            return sym;
        }
    }
}
