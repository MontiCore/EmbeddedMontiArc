package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

// creator if for clusterer factory
public abstract class Clusterer {
    public List<Set<ExpandedComponentInstanceSymbol>> createClusters(ExpandedComponentInstanceSymbol component, int numberOfClusters) {
        ClusteringAlgorithm clusteringAlgorithm = getClusteringAlgorithm();
        return clusteringAlgorithm.cluster(component, numberOfClusters);
    }
    public abstract ClusteringAlgorithm getClusteringAlgorithm();
}
