package de.monticore.lang.monticar.generator.middleware.clustering;

import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Comparator;
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
}
