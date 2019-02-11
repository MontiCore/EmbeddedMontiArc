package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

// product if for clustering factory
public interface ClusteringAlgorithm {
    List<Set<EMAComponentInstanceSymbol>> cluster(EMAComponentInstanceSymbol component, Object... args);

    //TODO: add arguments as typed state of the algorithms(instead of untyped)
    default List<Set<EMAComponentInstanceSymbol>> clusterWithState(EMAComponentInstanceSymbol component){
        Object[] args = getArgs();
        return cluster(component, args);
    }

    default Object[] getArgs(){
        return null;
    }

}
