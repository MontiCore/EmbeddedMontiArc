package de.monticore.lang.monticar.generator.middleware.cli;

import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.SpectralClusteringCliParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class ClusteringParameters {
    private Integer numberOfClusters;
    private Boolean flatten;
    private Integer flattenLevel;
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
        //Override numberOfClusters for all spectral clustering parameters
        if(getNumberOfClusters().isPresent()){
            Integer n = getNumberOfClusters().get();
            algorithmParameters.stream()
                    .filter(a -> a.getName().equals(AlgorithmCliParameters.TYPE_SPECTRAL_CLUSTERING))
                    .forEach(a -> ((SpectralClusteringCliParameters)a).setNumberOfClusters(n));
        }

        return algorithmParameters;
    }

    public boolean getFlatten(){
        return flatten == null ? false : flatten;
    }

    public Optional<Integer> getFlattenLevel() {
        return Optional.ofNullable(flattenLevel);
    }
}
