package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.monticar.generator.FileContent;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class ClusteringResultList extends ArrayList<ClusteringResult> {

    public ClusteringResultList() {
        super();
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
        this.sort(Comparator.comparing(ClusteringResult::getScore));
        return this.size() == 0 ? Optional.empty() : Optional.of(this.get(0));
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
