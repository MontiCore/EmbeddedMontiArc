package de.monticore.lang.monticar.generator.middleware.clustering.qualityMetric;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringResult;

public interface Metric {
    boolean higherIsBetter();
    double getScore(ClusteringResult clusteringResult);
}
