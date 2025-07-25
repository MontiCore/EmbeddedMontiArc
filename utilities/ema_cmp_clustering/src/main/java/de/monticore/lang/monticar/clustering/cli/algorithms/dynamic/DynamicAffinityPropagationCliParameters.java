/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AffinityPropagationCliParameters;
import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;

import java.util.Arrays;
import java.util.List;

public class DynamicAffinityPropagationCliParameters extends DynamicAlgorithmCliParameters {
    @Override
    public List<AlgorithmCliParameters> getAll() {
        return Arrays.asList(new AffinityPropagationCliParameters());
    }

    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_AFFINITY_PROPAGATION;
    }
}
