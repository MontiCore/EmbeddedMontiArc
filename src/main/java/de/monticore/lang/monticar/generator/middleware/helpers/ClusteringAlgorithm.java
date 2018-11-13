package de.monticore.lang.monticar.generator.middleware.helpers;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

// product if for clustering factory
public interface ClusteringAlgorithm {
    public List<Set<ExpandedComponentInstanceSymbol>> cluster(ExpandedComponentInstanceSymbol component, int numberOfClusters);
}
