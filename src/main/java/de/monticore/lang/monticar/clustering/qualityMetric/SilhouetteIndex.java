/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.qualityMetric;

import java.util.HashSet;
import java.util.Set;

public class SilhouetteIndex {

    private double[][] distanceMatrix;
    private int[] clusteringLabels;

    public SilhouetteIndex(double[][] distanceMatrix, int[] clusteringLabels) {
        this.distanceMatrix = distanceMatrix;
        this.clusteringLabels = clusteringLabels;
    }

    public double getSilhouetteScore(){
        double sum = 0d;
        for (int i = 0; i < clusteringLabels.length; i++) {
            sum += S(i);
        }

        return sum/clusteringLabels.length;
    }

    public double S(int o){
        double a = distA(o);
        double b = distB(o);
        if(a <= 0.0000000d && b <= 0.0000000d){
            return 0;
        }else{
            return (b-a)/Math.max(a,b);
        }
    }

    public double distA(int o){
        int clusterOfO = clusteringLabels[o];
        return averageDistToCluster(o, clusterOfO);
    }

    public double distB(int o){
        Set<Integer> processedLabels = new HashSet<>();
        int clusterOfO = clusteringLabels[o];
        double minDist = Double.MAX_VALUE;

        for (int label : clusteringLabels) {
            if(label != clusterOfO) {
                if (!processedLabels.contains(label)) {
                    double d = averageDistToCluster(o, label);
                    if (d < minDist) {
                        minDist = d;
                    }
                    processedLabels.add(label);
                }
            }
        }

        return minDist;
    }

    private double averageDistToCluster(int o, int clusterLabel){
        int numInCluster = 0;
        double sum = 0d;

        for (int i = 0; i < clusteringLabels.length; i++) {
            if(clusteringLabels[i] == clusterLabel){
                sum += distanceMatrix[o][i];
                numInCluster++;
            }
        }

        return sum/((double)numInCluster);
    }

}
