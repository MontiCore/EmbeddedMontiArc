/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.clustering.cli.algorithms.SpectralClusteringCliParameters;

import java.util.ArrayList;
import java.util.List;

public class DynamicSpectralClusteringCliParameters extends DynamicAlgorithmCliParameters {
    DynamicParameter numberOfClusters;
    DynamicParameter l;
    DynamicParameter sigma;

    public DynamicParameter getNumberOfClusters() {
        return numberOfClusters;
    }

    public void setNumberOfClusters(DynamicParameter numberOfClusters) {
        this.numberOfClusters = numberOfClusters;
    }

    public DynamicParameter getL() {
        return l;
    }

    public void setL(DynamicParameter l) {
        this.l = l;
    }

    public DynamicParameter getSigma() {
        return sigma;
    }

    public void setSigma(DynamicParameter sigma) {
        this.sigma = sigma;
    }

    @Override
    public List<AlgorithmCliParameters> getAll(){
        if(!isValid()){
            return new ArrayList<>();
        }

        List<AlgorithmCliParameters> res = new ArrayList<>();
        if(l == null || sigma == null){
            for (Integer n : numberOfClusters.getAllAsInt()) {
                res.add(new SpectralClusteringCliParameters(n));
            }
        }else{
            for(Integer a : numberOfClusters.getAllAsInt()){
                for(Integer b : l.getAllAsInt()){
                    for(Double c : sigma.getAll()){
                        res.add(new SpectralClusteringCliParameters(a, b, c));
                    }
                }
            }
        }
        return res;
    }


    @Override
    public boolean isValid(){
        if(numberOfClusters == null || !numberOfClusters.isValid()){
            return false;
        }

        if(l == null && sigma == null){
            return true;
        }

        if(l != null && l.isValid() && sigma != null && sigma.isValid()){
            return true;
        }

        return false;
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_SPECTRAL_CLUSTERING;
    }


}
