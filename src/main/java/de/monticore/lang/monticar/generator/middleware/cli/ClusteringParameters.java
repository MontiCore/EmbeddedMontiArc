package de.monticore.lang.monticar.generator.middleware.cli;

import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class ClusteringParameters {
    private Integer numberOfClusters;
    private ResultChoosingStrategy chooseBy = ResultChoosingStrategy.bestWithFittingN;
    private List<AlgorithmCliParameters> algorithmParameters = new ArrayList<>();

    public ClusteringParameters() {
    }

    public Optional<Integer> getNumberOfClusters() {
        return Optional.ofNullable(numberOfClusters);
    }

    public ResultChoosingStrategy getChooseBy() {
        return chooseBy;
    }

    public List<AlgorithmCliParameters> getAlgorithmParameters() {
        return algorithmParameters;
    }
}
