package de.monticore.lang.monticar.generator.middleware.clustering.qualityMetric;

import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringResult;

public class CommunicationCostMetric implements Metric {



    @Override
    public boolean higherIsBetter() {
        return false;
    }

    @Override
    public double getScore(ClusteringResult clusteringResult) {
        return AutomaticClusteringHelper.getTypeCostHeuristic(clusteringResult.getComponent(), clusteringResult.getClustering());
    }

}
