package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.generator.middleware.clustering.visualization.ModelVisualizer;
import de.monticore.lang.monticar.generator.middleware.impls.MiddlewareTagGenImpl;
import org.graphstream.graph.Graph;

import java.util.List;
import java.util.Set;

public class ClusteringResult {
    Double score = null;
    private EMAComponentInstanceSymbol component;
    private AlgorithmCliParameters parameters;
    private List<Set<EMAComponentInstanceSymbol>> clustering;
    private long durration;

    private ClusteringResult(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters, List<Set<EMAComponentInstanceSymbol>> clustering, long durration) {
        this.component = component;
        this.parameters = parameters;
        this.clustering = clustering;
        this.durration = durration;
    }

    public static ClusteringResult fromParameters(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters){
        long startTime = System.currentTimeMillis();
        List<Set<EMAComponentInstanceSymbol>> res = parameters.asClusteringAlgorithm().clusterWithState(component);
        long endTime = System.currentTimeMillis();
        return new ClusteringResult(component, parameters, res, endTime - startTime);
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

    private long getDurration() {
        return this.durration;
    }

    public boolean hasNumberOfClusters(int n){
        return getNumberOfClusters() == n;
    }

    public FileContent getTagFile(String fileName){
        FileContent res = new FileContent();
        res.setFileName(fileName);
        String prefix = "//Algorithm: " + this.getParameters().toString() + "\n" +
                        "//Number of clusters: " + this.getNumberOfClusters() + "\n" +
                        "//Score: " + this.getScore() + "\n" +
                        "//Durration in ms: " + this.getDurration() + "\n";

        String content = MiddlewareTagGenImpl.getFileContent(component, this.clustering);
        res.setFileContent(prefix + content);
        return res;
    }

    //TODO: refactor to File?
    public void saveVisualization(String path, String fileName){
        Graph g = ModelVisualizer.buildGraph(component, parameters.toString());
        ModelVisualizer.visualizeClustering(g, clustering, component);
        ModelVisualizer.saveGraphAsImage(g, path, fileName);
    }

}