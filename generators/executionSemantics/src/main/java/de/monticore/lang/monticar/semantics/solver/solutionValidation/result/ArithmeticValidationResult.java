/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation.result;

import java.util.Map;

public class ArithmeticValidationResult {

    private final String expression;
    private final boolean satisfiable;
    private final Map<String, Double> model;


    public ArithmeticValidationResult(String expression, boolean satisfiable, Map<String, Double> model) {
        this.expression = expression;
        this.satisfiable = satisfiable;
        this.model = model;
    }


}
