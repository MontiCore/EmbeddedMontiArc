/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import org.jscience.mathematics.number.Rational;

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class EMAMSpecificationSymbol extends MathExpressionSymbol {

    protected boolean resolvedComponentInstance = false;

    private Collection<EMAMSymbolicVariableSymbol> variables;

    private Collection<EMAMEquationSymbol> equations;

    private Collection<EMAMInitialValueSymbol> initialValues;

    private Collection<EMAMInitialGuessSymbol> initialGuesses;

    public EMAMSpecificationSymbol(Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMEquationSymbol> equations,
                                   Collection<EMAMInitialValueSymbol> initialValues, Collection<EMAMInitialGuessSymbol> initialGuesses) {
        this.variables = variables;
        this.equations = equations;
        this.initialValues = initialValues;
        this.initialGuesses = initialGuesses;
    }

    public EMAMSpecificationSymbol(Collection<EMAMSymbolicVariableSymbol> variables, Collection<EMAMEquationSymbol> equations) {
        this.variables = variables;
        this.equations = equations;
        this.initialValues = new HashSet<>();
        this.initialGuesses = new HashSet<>();
    }

    protected void resolveComponentInstance() {
        if (getEnclosingScope() == null)
            resolvedComponentInstance = false;
        else if (getEnclosingScope().getSpanningSymbol() == null)
            resolvedComponentInstance = false;
        else if (!getEnclosingScope().getSpanningSymbol().isPresent())
            resolvedComponentInstance = false;
        else if (getEnclosingScope().getSpanningSymbol().get() instanceof EMAComponentInstanceSymbol) {
            resolvedComponentInstance = true;
            EMAComponentInstanceSymbol component = (EMAComponentInstanceSymbol) getEnclosingScope().getSpanningSymbol().get();

            for (EMAPortInstanceSymbol port : component.getOutgoingPortInstances()) {
                // TODO maybe array
                EMAMSymbolicVariableSymbol var = new EMAMSymbolicVariableSymbol(port.getName());
                var.setPort(port);
                variables.add(var);

                if (port.isInitialGuessPresent()) {
                    EMAMInitialGuessSymbol initialGuess = convertInitialGuess(port);
                    Optional<EMAMInitialGuessSymbol> first = initialGuesses
                            .stream()
                            .filter(i -> i.getNameWithArray()
                                    .equals(initialGuess.getNameWithArray()))
                            .findFirst();
                    if (first.isPresent())
                        initialGuesses.remove(first.get());
                    initialGuesses.add(initialGuess);
                } else if (port.isInitialValuePresent()) {
                    EMAMInitialValueSymbol initialValue = convertInitialValue(port);
                    Optional<EMAMInitialValueSymbol> first = initialValues
                            .stream()
                            .filter(i -> i.getNameWithArray()
                                    .equals(initialValue.getNameWithArray()))
                            .findFirst();
                    if (first.isPresent())
                        initialValues.remove(first.get());
                    initialValues.add(initialValue);
                }
            }
        }
    }


    public Collection<EMAMEquationSymbol> getEquations() {
        return equations;
    }

    public void setEquations(Collection<EMAMEquationSymbol> equations) {
        this.equations = equations;
    }

    public void addEquation(EMAMEquationSymbol equation) {
        this.equations.add(equation);
    }

    public Collection<EMAMInitialValueSymbol> getInitialValues() {
        if (!resolvedComponentInstance) resolveComponentInstance();
        return initialValues;
    }

    public void setInitialValues(Collection<EMAMInitialValueSymbol> initialValues) {
        this.initialValues = initialValues;
    }

    public void addInitialValue(EMAMInitialValueSymbol initalValue) {
        this.initialValues.add(initalValue);
    }

    public Collection<EMAMInitialGuessSymbol> getInitialGuesses() {
        if (!resolvedComponentInstance) resolveComponentInstance();
        return initialGuesses;
    }

    public void setInitialGuesses(Collection<EMAMInitialGuessSymbol> initialGuesses) {
        this.initialGuesses = initialGuesses;
    }

    public void addInitialGuess(EMAMInitialGuessSymbol initialGuess) {
        this.initialGuesses.add(initialGuess);
    }

    public Collection<EMAMSymbolicVariableSymbol> getVariables() {
        if (!resolvedComponentInstance) resolveComponentInstance();
        return variables;
    }

    public void setVariables(Collection<EMAMSymbolicVariableSymbol> variables) {
        this.variables = variables;
    }

    public void addVariable(EMAMSymbolicVariableSymbol variable) {
        this.variables.add(variable);
    }

    @Override
    public String getTextualRepresentation() {
        StringBuilder result = new StringBuilder();
        for (EMAMSymbolicVariableSymbol variable : variables)
            result.append(String.format("symbolic Q %s;/n", variable.getTextualRepresentation()));
        for (EMAMInitialValueSymbol initialValue : initialValues)
            result.append(String.format("%s;/n", initialValue.getTextualRepresentation()));
        for (EMAMInitialGuessSymbol initialGuess : initialGuesses)
            result.append(String.format("%s;/n", initialGuess.getTextualRepresentation()));
        for (EMAMEquationSymbol equation : equations)
            result.append(String.format("%s;/n", equation.getTextualRepresentation()));
        return result.toString();
    }

    private static EMAMInitialValueSymbol convertInitialValue(EMAPortInstanceSymbol port) {
        EMAMInitialValueSymbol result = new EMAMInitialValueSymbol(port.getNameWithoutArrayBracketPart());
        ASTExpression initialValue = port.getInitialValue();
        if (port.isPartOfPortArray())
            result.setMathMatrixAccessOperatorSymbol(getIndex(port));

        MathExpressionSymbol symbolFromExpression = (MathExpressionSymbol) initialValue.getSymbol();
        result.setValue(symbolFromExpression);
        return result;
    }

    private static EMAMInitialGuessSymbol convertInitialGuess(EMAPortInstanceSymbol port) {
        EMAMInitialGuessSymbol result = new EMAMInitialGuessSymbol(port.getNameWithoutArrayBracketPart());
        ASTExpression initialGuess = port.getInitialGuess();
        if (port.isPartOfPortArray())
            result.setMathMatrixAccessOperatorSymbol(getIndex(port));

        MathExpressionSymbol symbolFromExpression = (MathExpressionSymbol) initialGuess.getSymbol();
        result.setValue(symbolFromExpression);
        return result;
    }

    private static MathMatrixAccessOperatorSymbol getIndex(EMAPortInstanceSymbol port) {
        String regex = ".*\\[(\\d+)\\]";
        Pattern pattern = Pattern.compile(regex);
        Matcher matcher = pattern.matcher(port.getName());

        String strIndex = "1";
        while (matcher.find())
            strIndex = matcher.group();


        MathMatrixAccessOperatorSymbol index = new MathMatrixAccessOperatorSymbol();
        MathMatrixAccessSymbol accessSymbol = new MathMatrixAccessSymbol(new MathNumberExpressionSymbol(Rational.valueOf(strIndex)));
        index.setMathMatrixAccessSymbols(Collections.singletonList(accessSymbol));

        return index;
    }
}
