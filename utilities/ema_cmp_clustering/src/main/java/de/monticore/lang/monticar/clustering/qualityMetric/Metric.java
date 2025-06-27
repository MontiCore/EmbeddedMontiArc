/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.qualityMetric;

import de.monticore.lang.monticar.clustering.ClusteringResult;

public interface Metric {
    boolean higherIsBetter();
    double getScore(ClusteringResult clusteringResult);
}
