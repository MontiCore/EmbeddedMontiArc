/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.monticar.cnnarch.helper.Calculator;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import org.jscience.mathematics.number.Rational;

import java.util.*;

public class ArchSimpleExpressionSymbol extends ArchExpressionSymbol {

    private MathExpressionSymbol mathExpression = null;
    private Object value = null;

    protected ArchSimpleExpressionSymbol() {
        super();
    }

    public Optional<MathExpressionSymbol> getMathExpression() {
        return Optional.ofNullable(mathExpression);
    }

    public void setMathExpression(MathExpressionSymbol mathExpression) {
        this.mathExpression = mathExpression;
    }

    @Override
    public Optional<Object> getValue() {
        return Optional.ofNullable(value);
    }

    protected void setValue(Object value){
        this.value = value;
    }

    @Override
    public void reset(){
        if (getMathExpression().isPresent()){
            if (getMathExpression().isPresent()){
                setValue(null);
                setUnresolvableParameters(null);
            }
        }
    }

    @Override
    public boolean isSimpleValue() {
        return true;
    }

    @Override
    public boolean isBoolean() {
        if (getMathExpression().isPresent() && !(getMathExpression().get().getRealMathExpressionSymbol() instanceof MathNameExpressionSymbol)){
            if (getMathExpression().get().getRealMathExpressionSymbol() instanceof MathCompareExpressionSymbol){
                return true;
            }
        }
        return getBooleanValue().isPresent();
    }

    @Override
    public boolean isNumber() {
        if (getMathExpression().isPresent()){
            MathExpressionSymbol mathExp = getMathExpression().get().getRealMathExpressionSymbol();
            if (mathExp instanceof MathArithmeticExpressionSymbol || mathExp instanceof MathNumberExpressionSymbol){
                return true;
            }
        }
        return getDoubleValue().isPresent();
    }

    @Override
    public boolean isTuple() {
        if (getMathExpression().isPresent() && !(getMathExpression().get().getRealMathExpressionSymbol() instanceof MathNameExpressionSymbol)){
            if (getMathExpression().get().getRealMathExpressionSymbol() instanceof TupleExpressionSymbol){
                return true;
            }
        }
        return getTupleValues().isPresent();
    }

    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        if (getMathExpression().isPresent()) {
            for (MathExpressionSymbol exp : Utils.createSubExpressionList(getMathExpression().get())) {
                if (exp instanceof MathNameExpressionSymbol) {
                    String name = ((MathNameExpressionSymbol) exp).getNameToAccess();
                    Optional<ParameterSymbol> parameter = getEnclosingScope().resolve(name, ParameterSymbol.KIND);
                    if (parameter.isPresent()) {
                        if (!allParameters.contains(parameter.get())) {
                            allParameters.add(parameter.get());
                            if (parameter.get().hasExpression()) {
                                if (!parameter.get().getExpression().isResolved()) {
                                    parameter.get().getExpression().computeUnresolvableParameters(unresolvableParameters, allParameters);
                                }
                            } else {
                                unresolvableParameters.add(parameter.get());
                            }
                        }
                    }
                    else {
                        unresolvableParameters.add(new ParameterSymbol.Builder()
                                .name(name)
                                .type(ParameterType.UNKNOWN)
                                .build());
                    }
                }
            }
        }
    }

    @Override
    public Set<ParameterSymbol> resolve() {
        if (!isResolved()) {
            if (isResolvable()) {
                if (getMathExpression().isPresent() && isResolvable()) {
                    Object value = computeValue();
                    setValue(value);
                }
            }
        }
        return getUnresolvableParameters();
    }

    private Object computeValue(){
        if (getMathExpression().get() instanceof MathNameExpressionSymbol){
            return computeValue((MathNameExpressionSymbol) getMathExpression().get());
        }
        else if (getMathExpression().get() instanceof TupleExpressionSymbol){
            return computeValue((TupleExpressionSymbol) getMathExpression().get());
        }
        else {
            Map<String, String> replacementMap = new HashMap<>();
            for (MathExpressionSymbol exp : Utils.createSubExpressionList(getMathExpression().get())) {
                if (exp instanceof MathNameExpressionSymbol) {
                    String name = ((MathNameExpressionSymbol) exp).getNameToAccess();
                    ParameterSymbol parameter = (ParameterSymbol) getEnclosingScope().resolve(name, ParameterSymbol.KIND).get();
                    parameter.getExpression().resolveOrError();

                    replacementMap.put(name, parameter.getExpression().getTextualRepresentation());
                }
            }

            String resolvedString = Utils.replace(getTextualRepresentation(), replacementMap);
            return Calculator.getInstance().calculate(resolvedString);
        }
    }

    private List<Object> computeValue(TupleExpressionSymbol tupleExpression){
        List<Object> valueList = new ArrayList<>();
        for (MathExpressionSymbol mathExp : tupleExpression.getExpressions()){
            if (mathExp instanceof MathNameExpressionSymbol){
                valueList.add(computeValue((MathNameExpressionSymbol) mathExp));
            }
            else {
                ArchSimpleExpressionSymbol temp = ArchSimpleExpressionSymbol.of(mathExp);
                temp.setEnclosingScope(getEnclosingScope().getAsMutableScope());
                temp.resolveOrError();
                valueList.add(temp.getValue().get());
                getEnclosingScope().getAsMutableScope().remove(temp);
            }
        }
        return valueList;
    }

    private Object computeValue(MathNameExpressionSymbol mathExpression){
        String name = mathExpression.getNameToAccess();
        ParameterSymbol variable = (ParameterSymbol) getEnclosingScope().resolve(name, ParameterSymbol.KIND).get();
        variable.getExpression().resolveOrError();

        return variable.getExpression().getValue().get();
    }

    @Override
    public String getTextualRepresentation() {
        if (isResolved()){
            if (isTuple()){
                return Utils.createTupleTextualRepresentation(getTupleValues().get(), Object::toString);
            }
            else {
                return getValue().get().toString();
            }
        }
        else {
            return getMathExpression().get().getTextualRepresentation();
        }
    }

    @Override
    public Optional<List<List<ArchSimpleExpressionSymbol>>> getElements(){
        return Optional.of(Collections.singletonList(Collections.singletonList(this)));
    }

    @Override
    public boolean isResolved() {
        return getValue().isPresent() || !getMathExpression().isPresent();
    }

    @Override
    protected void putInScope(Scope scope) {
        super.putInScope(scope);
        if (getMathExpression().isPresent()){
            for (MathExpressionSymbol exp : Utils.createSubExpressionList(getMathExpression().get())) {
                Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(exp.getName());
                if (symbolsInScope == null || !symbolsInScope.contains(exp)) {
                    scope.getAsMutableScope().add(exp);
                }
            }
        }
    }

    @Override
    public ArchSimpleExpressionSymbol preResolveDeepCopy(){
        ArchSimpleExpressionSymbol copy = new ArchSimpleExpressionSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }
        if (getMathExpression().isPresent()){
            copy.setMathExpression(Utils.copy(getMathExpression().get()));
        }
        else {
            copy.setValue(value);
        }
        return copy;
    }


    public static ArchSimpleExpressionSymbol of(Rational value){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        if (value.getDivisor().intValue() == 1){
            res.setValue(value.getDividend().intValue());
        }
        else {
            res.setValue(value.doubleValue());
        }
        return res;
    }

    public static ArchSimpleExpressionSymbol of(int value){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(value);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(double value){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(value);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(boolean value){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(value);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(String value){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(value);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(MathExpressionSymbol expressions){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setMathExpression(expressions);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(List<Object> tupleValues){
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setValue(tupleValues);
        return res;
    }

    public static ArchSimpleExpressionSymbol of(ParameterSymbol variable){
        MathExpressionSymbol exp = new MathNameExpressionSymbol(variable.getName());
        ArchSimpleExpressionSymbol res = new ArchSimpleExpressionSymbol();
        res.setMathExpression(exp);
        return res;
    }
}
