package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;

import java.util.List;

public abstract class AlgorithmCliParameters {
    public static final String TYPE_SPECTRAL_CLUSTERING = "SpectralClustering";
    public static final String TYPE_UNKOWN = "Unkown";
    public static final String TYPE_DBSCAN = "DBScan";
    public static final String TYPE_MARKOV = "Markov";
    protected String name;

    public AlgorithmCliParameters() {
    }

    public String getName() {
        return name;
    }

    public abstract ClusteringAlgorithm asClusteringAlgorithm();

    public abstract List<Object> asAlgorithmArgs();

    public abstract boolean isValid();
}
