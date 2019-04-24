package de.monticore.lang.monticar.clustering.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.AutomaticClusteringHelper;

import java.util.*;

public class MonteCarloIntegration{
    private double[] averages;
    private double[] costs;
    private int iterations;
    private int numberOfClusters;
    private MonteCarloStrategy clusteringDelegate;

    public MonteCarloIntegration(int iterations, int numberOfClusters){
        this.iterations = iterations;
        this.numberOfClusters = numberOfClusters;
        this.averages = new double[this.iterations];
        this.costs = new double[this.iterations];
        this.clusteringDelegate = new MonteCarloRandomStrategy();
    }

    public double[] getAverages() {
        return averages;
    }

    public int getIterations() {
        return iterations;
    }

    public int getNumberOfClusters() {
        return numberOfClusters;
    }

    public double[] getCosts() {
        return costs;
    }

    public double simulate(EMAComponentInstanceSymbol componentInstanceSymbol) {
        //EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);
        double sum = 0;

        for (int i = 0; i < iterations; i++) {
            // Let's Random cluster the model
            List<Set<EMAComponentInstanceSymbol>> clusters = randomClustering(componentInstanceSymbol, numberOfClusters);

            costs[i] = AutomaticClusteringHelper.getTypeCostHeuristic(componentInstanceSymbol, clusters);
            //iterate through all clusters and add all cost of the ROS Tags between clusters
            sum = getCostAtN(i+1, costs);
            averages[i]=sum;
        }
        // Plot(x,y) would be Plot(iterations, getCostAtN(i+1
        return sum;
    }



    // getting the maximum value
    public double getMaxValue() {
        double maxValue = costs[0];
        for (int i = 1; i < costs.length; i++) {
            if (costs[i] > maxValue) {
                maxValue = costs[i];
            }
        }
        return maxValue;
    }

    // getting the miniumum value
    public double getMinValue() {
        double minValue = costs[0];
        for (int i = 1; i < costs.length; i++) {
            if (costs[i] < minValue) {
                minValue = costs[i];
            }
        }
        return minValue;
    }

    public List<Set<EMAComponentInstanceSymbol>> randomClustering(EMAComponentInstanceSymbol componentInstanceSymbol, int numberOfClusters) {
        return clusteringDelegate.randomClustering(componentInstanceSymbol, numberOfClusters);
    }

    // iteration step n, costs: Array of all costs before
    public static double getCostAtN(int n, double[] array) {
        double sum = 0;

        for (int i = 0; i < n ; i++) {
            sum += array[i];
        }

        return sum /n;
    }
}