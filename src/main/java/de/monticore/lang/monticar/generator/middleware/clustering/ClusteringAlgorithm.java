package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

// product if for clustering factory
public interface ClusteringAlgorithm {
    public List<Set<EMAComponentInstanceSymbol>> cluster(EMAComponentInstanceSymbol component, Object... args);
}
