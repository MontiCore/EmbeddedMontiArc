package de.monticore.lang.monticar.generator.middleware.clustering;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.generator.middleware.clustering.visualization.ModelVisualizer;
import de.monticore.lang.monticar.generator.middleware.impls.MiddlewareTagGenImpl;
import de.se_rwth.commons.logging.Log;
import org.graphstream.graph.Graph;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ClusteringResult {
    Double score = null;
    private EMAComponentInstanceSymbol component;
    private AlgorithmCliParameters parameters;
    private List<Set<EMAComponentInstanceSymbol>> clustering;
    private long duration;
    private int componentNumber;
    private boolean valid;

    private ClusteringResult(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters,
                             List<Set<EMAComponentInstanceSymbol>> clustering, long duration, int componentNumber, boolean valid) {
        this.component = component;
        this.parameters = parameters;
        this.clustering = clustering;
        this.duration = duration;
        this.componentNumber = componentNumber;
        this.valid = valid;
    }

    public static ClusteringResult fromParameters(EMAComponentInstanceSymbol component, AlgorithmCliParameters parameters) {
        List<Set<EMAComponentInstanceSymbol>> res;
        long startTime = System.currentTimeMillis();

        try {
            res = parameters.asClusteringAlgorithm().clusterWithState(component);
        } catch (Exception e) {
            Log.warn("Marking this result as invalid. Error clustering the component.", e);
            return new ClusteringResult(component, parameters, new ArrayList<>(), -1, component.getSubComponents().size(), false);
        }

        long endTime = System.currentTimeMillis();
        boolean curValid = true;

        int clustersBefore = res.size();
        res.removeIf(Set::isEmpty);
        if (clustersBefore != res.size()) {
            Log.warn("Removed " + (clustersBefore - res.size()) + " empty clusters for algorithm " + parameters.toString());
        }

        Optional<Integer> expClusters = parameters.expectedClusterCount();
        if(expClusters.isPresent() && !expClusters.get().equals(res.size())){
            curValid = false;
            Log.warn("Marking this result as invalid. The actual number of clusters(" + res.size() + ") does not equal the expected number(" + expClusters.get() +")");
        }

        int componentNumber = 0;
        for (Set<EMAComponentInstanceSymbol> cluster : res) {
            componentNumber += cluster.size();
        }
        return new ClusteringResult(component, parameters, res, endTime - startTime, componentNumber, curValid);
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

    private long getDuration() {
        return this.duration;
    }

    public int getComponentNumber() {
        return componentNumber;
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
                        "//Durration in ms: " + this.getDuration() + "\n";

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

    public void saveAsJson(String path,String filename){
        JsonParser parser = new JsonParser();
        File dir = new File(path);
        File file = new File(dir, filename);
        JsonArray jsonArray = new JsonArray();
        if (file.exists() && !file.isDirectory()) {
            try {
                jsonArray = (JsonArray) parser.parse(new FileReader(file));
                JsonObject result = createJsonResultObject();
                jsonArray.add(result);
            } catch (IOException e) {
                Log.warn("Could not open clustering result file " + filename);
            }
        }else {
            JsonObject result = createJsonResultObject();
            jsonArray.add(result);
        }
        try {
            file.getParentFile().mkdirs();
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(jsonArray.toString());
            fileWriter.flush();
        }catch (IOException io){
            Log.warn("Could not write clustering results to json file " + filename);
        }
    }

    private JsonObject createJsonResultObject() {
        JsonObject result = new JsonObject();
        result.addProperty("Algorithm", this.getParameters().toString());
        result.addProperty("NumberOfClusters", this.getNumberOfClusters());
        result.addProperty("Score", this.getScore());
        result.addProperty("DurationInMs", this.getDuration());
        result.addProperty("ComponentNumber", this.getComponentNumber());
        return result;
    }

    public boolean isValid() {
        return valid;
    }

    public void setValid(boolean valid) {
        this.valid = valid;
    }
}