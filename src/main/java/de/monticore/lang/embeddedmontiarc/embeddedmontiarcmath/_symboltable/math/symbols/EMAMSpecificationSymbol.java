/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.Collection;
import java.util.HashSet;

public class EMAMSpecificationSymbol extends MathExpressionSymbol {

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


    public Collection<EMAMEquationSymbol> getEquations() {
        return equations;
    }

    public void setEquations(Collection<EMAMEquationSymbol> equations) {
        this.equations = equations;
    }

    public Collection<EMAMInitialValueSymbol> getInitialValues() {
        return initialValues;
    }

    public void setInitialValues(Collection<EMAMInitialValueSymbol> initialValues) {
        this.initialValues = initialValues;
    }

    public Collection<EMAMInitialGuessSymbol> getInitialGuesses() {
        return initialGuesses;
    }

    public void setInitialGuesses(Collection<EMAMInitialGuessSymbol> initialGuesses) {
        this.initialGuesses = initialGuesses;
    }

    public Collection<EMAMSymbolicVariableSymbol> getVariables() {
        return variables;
    }

    public void setVariables(Collection<EMAMSymbolicVariableSymbol> variables) {
        this.variables = variables;
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
}
