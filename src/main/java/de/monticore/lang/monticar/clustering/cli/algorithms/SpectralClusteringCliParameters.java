/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms;

import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.algorithms.SpectralClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.algorithms.SpectralClusteringBuilder;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class SpectralClusteringCliParameters extends AlgorithmCliParameters {
    private Integer numberOfClusters;
    private Integer l;
    private Double sigma;

    public SpectralClusteringCliParameters() {
    }

    public SpectralClusteringCliParameters(int numberOfClusters) {
        this.numberOfClusters = numberOfClusters;
    }

    public SpectralClusteringCliParameters(int numberOfClusters, int l, double sigma) {
        this.numberOfClusters = numberOfClusters;
        this.l = l;
        this.sigma = sigma;
    }

    @Override
    public String getName() {
        return TYPE_SPECTRAL_CLUSTERING;
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        SpectralClusteringAlgorithm clusteringAlgorithm = new SpectralClusteringAlgorithm();
        clusteringAlgorithm.setArgs(asAlgorithmArgs());
        return clusteringAlgorithm;
    }

    @Override
    public Object[] asAlgorithmArgs(){
        ArrayList<Object> res = new ArrayList<>();

        if(!isValid()){
            Log.error("SpectralClusteringCliParameters: The numberOfClusters parameter is mandatory but unset!");
            return res.toArray();
        }

        res.add(SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS);
        res.add(numberOfClusters);

        if(l != null){
            res.add(SpectralClusteringBuilder.SpectralParameters.SPECTRAL_L);
            res.add(l);
        }

        if(sigma != null){
            res.add(SpectralClusteringBuilder.SpectralParameters.SPECTRAL_SIGMA);
            res.add(sigma);
        }

        return res.toArray();
    }

    @Override
    public boolean isValid() {
        return numberOfClusters != null;
    }

    @Override
    public Optional<Integer> expectedClusterCount() {
        return Optional.of(getNumberOfClusters().get());
    }

    public Optional<Integer> getNumberOfClusters() {
        return Optional.ofNullable(numberOfClusters);
    }

    public Optional<Integer> getL() {
        return Optional.ofNullable(l);
    }

    public Optional<Double> getSigma() {
        return Optional.ofNullable(sigma);
    }

    public void setNumberOfClusters(Integer numberOfClusters) {
        this.numberOfClusters = numberOfClusters;
    }

    public void setL(Integer l) {
        this.l = l;
    }

    public void setSigma(Double sigma) {
        this.sigma = sigma;
    }

    @Override
    public String toString() {
        return "SpectralClustering{" +
                "numberOfClusters=" + numberOfClusters +
                ", l=" + l +
                ", sigma=" + sigma +
                '}';
    }
}
