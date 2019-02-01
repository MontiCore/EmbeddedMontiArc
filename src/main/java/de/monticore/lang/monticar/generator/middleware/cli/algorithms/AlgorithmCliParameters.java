package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

public class AlgorithmCliParameters {
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
}
