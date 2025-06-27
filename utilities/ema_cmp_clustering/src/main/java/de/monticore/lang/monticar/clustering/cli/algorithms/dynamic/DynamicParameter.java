/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import java.util.ArrayList;
import java.util.List;

public abstract class DynamicParameter{

    public List<Integer> getAllAsInt(){
        List<Double> asDouble = getAll();
        List<Integer> res = new ArrayList<>();
        asDouble.forEach(d -> res.add(d.intValue()));
        return res;
    }

    public abstract List<Double> getAll();

    public abstract boolean isValid();
}
