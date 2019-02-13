package de.monticore.lang.monticar.generator.middleware.cli.algorithms.dynamic;

import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.UnknownAlgorithmCliParameters;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DynamicUnknownAlgorithmCliParameters extends DynamicAlgorithmCliParameters {

    public DynamicUnknownAlgorithmCliParameters() {
    }

    @Override
    public List<AlgorithmCliParameters> getAll() {
        return Arrays.asList(new UnknownAlgorithmCliParameters());
    }

    @Override
    public boolean isValid() {
        return false;
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_UNKOWN;
    }


}
