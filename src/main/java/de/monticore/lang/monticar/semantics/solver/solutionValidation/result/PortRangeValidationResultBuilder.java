/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation.result;

import java.util.HashMap;
import java.util.Map;

public final class PortRangeValidationResultBuilder {
    private String name;
    private boolean upper;
    private boolean satisfiable;
    private Map<String, Double> model = new HashMap<>();

    private PortRangeValidationResultBuilder() {
    }

    public static PortRangeValidationResultBuilder aPortRangeValidation() {
        return new PortRangeValidationResultBuilder();
    }

    public PortRangeValidationResultBuilder setName(String name) {
        this.name = name;
        return this;
    }

    public PortRangeValidationResultBuilder setUpper(boolean upper) {
        this.upper = upper;
        return this;
    }

    public PortRangeValidationResultBuilder setSatisfiable(boolean satisfiable) {
        this.satisfiable = satisfiable;
        return this;
    }

    public PortRangeValidationResultBuilder addParameterWithValue(String varName, double value) {
        model.put(varName, value);
        return this;
    }

    public PortRangeValidationResultBuilder setModel(Map<String, Double> model) {
        this.model = model;
        return this;
    }

    public PortRangeValidationResult build() {
        return new PortRangeValidationResult(name, upper, satisfiable, model);
    }
}
