/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation.result;

import java.util.HashMap;
import java.util.Map;

public final class ArithmeticValidationResultBuilder {
    private String expression;
    private boolean satisfiable;
    private Map<String, Double> model = new HashMap<>();

    private ArithmeticValidationResultBuilder() {
    }

    public static ArithmeticValidationResultBuilder anArithmeticValidation() {
        return new ArithmeticValidationResultBuilder();
    }

    public ArithmeticValidationResultBuilder setExpression(String expression) {
        this.expression = expression;
        return this;
    }

    public ArithmeticValidationResultBuilder setSatisfiable(boolean satisfiable) {
        this.satisfiable = satisfiable;
        return this;
    }

    public ArithmeticValidationResultBuilder addParameterWithValue(String varName, double value) {
        model.put(varName, value);
        return this;
    }

    public ArithmeticValidationResultBuilder setModel(Map<String, Double> model) {
        this.model = model;
        return this;
    }

    public ArithmeticValidationResult build() {
        return new ArithmeticValidationResult(expression, satisfiable, model);
    }
}
