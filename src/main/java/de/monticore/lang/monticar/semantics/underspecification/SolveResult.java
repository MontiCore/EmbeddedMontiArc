/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.underspecification;

import java.util.Map;

public class SolveResult {

    private final boolean satisfiable;
    private final Map<String, Double> model;

    public SolveResult(boolean satisfiable, Map<String, Double> model) {
        this.satisfiable = satisfiable;
        this.model = model;
    }

    public boolean isSatisfiable() {
        return satisfiable;
    }

    public Map<String, Double> getModel() {
        return model;
    }


}
