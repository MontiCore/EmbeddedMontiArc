/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

public class ArithmeticValidationViewModel {

    private final String expression;
    private final String validation;

    public ArithmeticValidationViewModel(String expression, String validation) {
        this.expression = expression;
        this.validation = validation;
    }

    public String getExpression() {
        return expression;
    }

    public String getValidation() {
        return validation;
    }
}
