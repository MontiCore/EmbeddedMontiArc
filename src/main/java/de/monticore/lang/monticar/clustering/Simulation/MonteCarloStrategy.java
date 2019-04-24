package de.monticore.lang.monticar.clustering.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.List;
import java.util.Set;

public interface MonteCarloStrategy {

    List<Set<EMAComponentInstanceSymbol>> randomClustering(EMAComponentInstanceSymbol componentSymbol, int numberOfClusters);


}
