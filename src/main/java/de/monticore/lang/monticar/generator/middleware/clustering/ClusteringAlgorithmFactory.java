package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.KMeansAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.se_rwth.commons.logging.Log;

public class ClusteringAlgorithmFactory {

    private ClusteringAlgorithmFactory(){

    }

    public static ClusteringAlgorithm getFromKind(ClusteringKind kind){
        switch (kind){
            case SPECTRAL_CLUSTERER: return new SpectralClusteringAlgorithm();
            case KMEANS: return new KMeansAlgorithm();
            default: Log.error("0x1D54C: No clustering algorithm found for ClusteringKind " + kind);
        }
        return null;
    }


}
