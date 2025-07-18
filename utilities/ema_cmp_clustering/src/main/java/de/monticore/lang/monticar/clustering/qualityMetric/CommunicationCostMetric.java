/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.qualityMetric;

import de.monticore.lang.monticar.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.clustering.ClusteringResult;

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
