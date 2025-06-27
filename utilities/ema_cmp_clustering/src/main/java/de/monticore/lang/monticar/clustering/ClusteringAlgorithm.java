/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

// product if for clustering factory
public interface ClusteringAlgorithm {
    List<Set<EMAComponentInstanceSymbol>> cluster(ClusteringInput clusteringInput, Object... args);

    //TODO: add arguments as typed state of the algorithms(instead of untyped)
    default List<Set<EMAComponentInstanceSymbol>> clusterWithState(ClusteringInput clusteringInput){
        Object[] args = getArgs();
        return cluster(clusteringInput, args);
    }

    default Object[] getArgs(){
        return null;
    }

}
