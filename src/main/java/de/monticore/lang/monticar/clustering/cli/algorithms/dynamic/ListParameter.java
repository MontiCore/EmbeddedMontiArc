/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import java.util.Arrays;
import java.util.List;

public class ListParameter extends DynamicParameter {
    private List<Double> values;

    public ListParameter() {
    }

    public ListParameter(List<Double> values) {
        this.values = values;
    }

    public ListParameter(Double value) {
        this.values = Arrays.asList(value);
    }

    @Override
    public List<Double> getAll() {
        return values;
    }

    @Override
    public boolean isValid() {
        return !values.isEmpty();
    }
}
