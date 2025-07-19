/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.ClusteringInput;

import java.util.List;
import java.util.Set;

public interface MonteCarloStrategy {

    List<Set<EMAComponentInstanceSymbol>> randomClustering(ClusteringInput input, int numberOfClusters);


}
