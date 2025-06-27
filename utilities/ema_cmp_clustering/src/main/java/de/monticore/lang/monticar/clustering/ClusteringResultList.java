/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.clustering.qualityMetric.Metric;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class ClusteringResultList extends ArrayList<ClusteringResult> {

    public ClusteringResultList() {
        super();
    }

    public static ClusteringResultList fromParametersList(EMAComponentInstanceSymbol emaComponentInstance, List<AlgorithmCliParameters> algoParams, Metric metric) {
        ClusteringResultList res = new ClusteringResultList();
        ClusteringInput clusteringInput = new ClusteringInput(emaComponentInstance);
        //create AdjacencyMatrix to make execution speed comparision fairer
        clusteringInput.getAdjacencyMatrix();

        for (int i = 0; i < algoParams.size(); i++) {
            System.out.println("Clustering with algorithm " + (i + 1) + "/" + algoParams.size() + ": " + algoParams.get(i).toString());
            ClusteringResult result = ClusteringResult.fromParameters(clusteringInput, algoParams.get(i), metric);
            if (result.isValid()) {
                res.add(result);
            } else {
                Log.warn("Ignoring the result! It is invalid!");
            }
        }
        return res;
    }

    public Optional<ClusteringResult> getBestResultWithFittingN(int n){
        ClusteringResultList filteredList = new ClusteringResultList();
        for(ClusteringResult c : this){
            if(!c.hasNumberOfClusters(n)){
                Log.warn("Not the right number of clusters! Ignoring the clustering from algorithm " + c.getParameters().toString());
            }else{
                filteredList.add(c);
            }
        }
        return filteredList.getBestResultOverall();

    }

    public Optional<ClusteringResult> getBestResultOverall(){
        if(this.isEmpty()){
            return Optional.empty();
        }

        ClusteringResultList copiedList = new ClusteringResultList();
        copiedList.addAll(this);
        copiedList.sort(Comparator.comparing(ClusteringResult::getScore));

        int index = copiedList.get(0).getMetric().higherIsBetter() ? copiedList.size() - 1 : 0;
        return Optional.of(copiedList.get(index));
    }

    public List<FileContent> getAllTagFiles(String baseName){
        List<FileContent> res = new ArrayList<>();
        int i = 1;
        for (ClusteringResult r : this) {
            FileContent tagFile = r.getTagFile(baseName + "_" + i + "_" + r.getParameters().getName() + ".tag");
            res.add(tagFile);
            i++;
        }
        return res;
    }

    //TODO: refactor to List<File>?
    public void saveAllVisualizations(String path, String baseName){
        int i = 1;
        for (ClusteringResult r : this){
            r.saveVisualization(path, baseName + "_" + i + "_" + r.getParameters().getName());
            i++;
        }
    }
}
