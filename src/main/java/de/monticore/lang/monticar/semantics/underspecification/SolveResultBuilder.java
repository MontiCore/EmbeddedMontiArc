/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.underspecification;

import java.util.HashMap;
import java.util.Map;

public final class SolveResultBuilder {
    private boolean satisfiable;
    private Map<String, Double> model = new HashMap<>();

    private SolveResultBuilder() {
    }

    public static SolveResultBuilder aSolveResult() {
        return new SolveResultBuilder();
    }

    public SolveResultBuilder setSatisfiable(boolean satisfiable) {
        this.satisfiable = satisfiable;
        return this;
    }

    public SolveResultBuilder addParameterWithValue(String varName, double value) {
        model.put(varName, value);
        return this;
    }

    public SolveResultBuilder setModel(Map<String, Double> model) {
        this.model = model;
        return this;
    }

    public SolveResult build() {
        return new SolveResult(satisfiable, model);
    }
}
