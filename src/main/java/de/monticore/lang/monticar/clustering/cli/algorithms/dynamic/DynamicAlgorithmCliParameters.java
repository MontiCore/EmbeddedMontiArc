/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;

import java.util.Arrays;
import java.util.List;

public abstract class DynamicAlgorithmCliParameters {
    public abstract List<AlgorithmCliParameters> getAll();
    public abstract boolean isValid();
    public abstract String getName();

    protected List<Double> getValuesOrSingleElement(DynamicParameter dynamicParameter, Double elem){
        if(dynamicParameter == null){
            return Arrays.asList(elem);
        }else{
            return dynamicParameter.getAll();
        }
    }
}
