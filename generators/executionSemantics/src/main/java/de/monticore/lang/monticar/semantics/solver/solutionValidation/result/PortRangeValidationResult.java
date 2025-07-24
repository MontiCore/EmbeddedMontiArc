/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation.result;

import java.util.Map;

public class PortRangeValidationResult {

    private final String name;
    private final boolean upper;
    private final boolean satisfiable;
    private final Map<String, Double> model;

    public PortRangeValidationResult(String name, boolean upper, boolean satisfiable, Map<String, Double> model) {
        this.name = name;
        this.upper = upper;
        this.satisfiable = satisfiable;
        this.model = model;
    }
}
