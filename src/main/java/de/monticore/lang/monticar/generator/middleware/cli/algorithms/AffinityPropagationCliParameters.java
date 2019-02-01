package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.AffinityPropagationAlgorithm;

import java.util.ArrayList;
import java.util.List;

public class AffinityPropagationCliParameters extends AlgorithmCliParameters{

    public AffinityPropagationCliParameters() {
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        return new AffinityPropagationAlgorithm();
    }

    @Override
    public List<Object> asAlgorithmArgs(){
        return new ArrayList<>();
    }

    @Override
    public boolean isValid() {
        return true;
    }
}