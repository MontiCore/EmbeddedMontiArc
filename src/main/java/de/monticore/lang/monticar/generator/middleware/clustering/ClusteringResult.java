package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;

import java.util.List;
import java.util.Set;

public class ClusteringResult {
    Double score = null;
    private EMAComponentInstanceSymbol component;

    private AlgorithmCliParameters parameters;
    private List<Set<EMAComponentInstanceSymbol>> clustering;

    private ClusteringResult(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters, List<Set<EMAComponentInstanceSymbol>> clustering) {
        this.component = component;
        this.parameters = parameters;
        this.clustering = clustering;
    }

    public static ClusteringResult fromParameters(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters){
        List<Set<EMAComponentInstanceSymbol>> res = parameters.asClusteringAlgorithm().clusterWithState(component);
        return new ClusteringResult(component, parameters, res);
    }

    public double getScore(){
        if(score == null){
            score = AutomaticClusteringHelper.getTypeCostHeuristic(component, clustering);
        }
        return score;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }

    public AlgorithmCliParameters getParameters() {
        return parameters;
    }

    public List<Set<EMAComponentInstanceSymbol>> getClustering() {
        return clustering;
    }

    public int getNumberOfClusters(){
        return clustering.size();
    }

    public boolean hasNumberOfClusters(int n){
        return getNumberOfClusters() == n;
    }
}
