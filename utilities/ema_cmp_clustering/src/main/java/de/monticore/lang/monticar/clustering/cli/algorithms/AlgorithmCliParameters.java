/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms;

import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;

import java.util.Optional;

public abstract class AlgorithmCliParameters {
    public static final String TYPE_SPECTRAL_CLUSTERING = "SpectralClustering";
    public static final String TYPE_UNKOWN = "Unkown";
    public static final String TYPE_DBSCAN = "DBScan";
    public static final String TYPE_MARKOV = "Markov";
    public static final String TYPE_AFFINITY_PROPAGATION = "AffinityPropagation";
    protected String name;

    public AlgorithmCliParameters() {
    }

    public String getName() {
        return name;
    }

    public abstract ClusteringAlgorithm asClusteringAlgorithm();

    public abstract Object[] asAlgorithmArgs();

    public abstract boolean isValid();

    public Optional<Integer> expectedClusterCount(){
        return Optional.empty();
    }
}
