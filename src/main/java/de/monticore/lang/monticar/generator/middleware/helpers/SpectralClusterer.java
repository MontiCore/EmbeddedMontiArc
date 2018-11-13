package de.monticore.lang.monticar.generator.middleware.helpers;

// creator for spectral clusterer
public class SpectralClusterer extends Clusterer {
    @Override
    public ClusteringAlgorithm getClusteringAlgorithm() {
        return new SpectralClusteringAlgorithm();
    }
}

