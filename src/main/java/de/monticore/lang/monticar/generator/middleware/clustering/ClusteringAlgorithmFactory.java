package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.AffinityPropagationAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.DBSCANClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.MarkovClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.se_rwth.commons.logging.Log;

public class ClusteringAlgorithmFactory {

    private ClusteringAlgorithmFactory(){

    }

    public static ClusteringAlgorithm getFromKind(ClusteringKind kind){
        switch (kind){
            case SPECTRAL_CLUSTERER: return new SpectralClusteringAlgorithm();
            case MARKOV_CLUSTERER: return new MarkovClusteringAlgorithm();
            case DBSCAN_CLUSTERER: return new DBSCANClusteringAlgorithm();
            case AFFINITY_CLUSTERER: return new AffinityPropagationAlgorithm();
            default: Log.error("0x1D54C: No clustering algorithm found for ClusteringKind " + kind);
        }
        return null;
    }


}
