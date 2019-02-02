package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;

public class UnknownAlgorithmCliParameters extends AlgorithmCliParameters {

    public UnknownAlgorithmCliParameters() {
    }

    @Override
    public String getName() {
        return TYPE_UNKOWN;
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        return null;
    }

    @Override
    public Object[] asAlgorithmArgs() {
        return null;
    }

    @Override
    public boolean isValid() {
        return false;
    }
}
