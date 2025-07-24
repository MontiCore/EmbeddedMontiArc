/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.clustering.cli.algorithms.MarkovCliParameters;

import java.util.ArrayList;
import java.util.List;

public class DynamicMarkovCliParameters extends DynamicAlgorithmCliParameters {
    private DynamicParameter max_residual;
    private DynamicParameter gamma_exp;
    private DynamicParameter loop_gain;
    private DynamicParameter zero_max;

    @Override
    public List<AlgorithmCliParameters> getAll() {
        List<AlgorithmCliParameters> res = new ArrayList<>();
        for(Double mr : getValuesOrSingleElement(max_residual, null)){
            for(Double ge : getValuesOrSingleElement(gamma_exp, null)){
                for(Double lg : getValuesOrSingleElement(loop_gain, null)){
                    for(Double zm : getValuesOrSingleElement(zero_max, null)){
                        res.add(new MarkovCliParameters(mr, ge, lg, zm));
                    }
                }
            }
        }
        return res;
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_MARKOV;
    }

    @Override
    public boolean isValid() {
        return true;
    }
}
