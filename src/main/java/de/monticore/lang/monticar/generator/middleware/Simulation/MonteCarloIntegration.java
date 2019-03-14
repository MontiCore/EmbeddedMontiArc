package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;

import java.util.*;

public class MonteCarloIntegration {
    private double[] averages;
    private double[] costs;
    private int iterations;
    private int numberOfClusters;

    public MonteCarloIntegration(int iterations, int numberOfClusters){
        this.iterations = iterations;
        this.numberOfClusters = numberOfClusters;
        this.averages = new double[this.iterations];
        this.costs = new double[this.iterations];
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
             /*
             // This would be with Spectral Clustering
             SpectralClusteringAlgorithm spectralClusteringAlgorithm = new SpectralClusteringAlgorithm();
             Object[] params = new Object[]{SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS, numberOfClusters};
             List<Set<EMAComponentInstanceSymbol>> clusters = spectralClusteringAlgorithm.cluster(componentInstanceSymbol, params);
             */

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

    public static int randomNumberInRange(int min, int max) {
        Random random = new Random();
        return random.nextInt((max - min) + 1) + min;
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

    public static List<Set<EMAComponentInstanceSymbol>> randomClustering(EMAComponentInstanceSymbol componentInstanceSymbol, int numberOfClusters) {
        List<Set<EMAComponentInstanceSymbol>> clusters = new ArrayList<>();

        for (int i = 0; i < numberOfClusters; i++) {
            clusters.add(new HashSet<>());
        }

        // All subcomponents of the Symbol
        Collection<EMAComponentInstanceSymbol> subcomponents = componentInstanceSymbol.getSubComponents();

        // Put subcomponents into an ArrayList
        ArrayList<EMAComponentInstanceSymbol> arrayListSubComponent = new ArrayList<>();
        for (EMAComponentInstanceSymbol subcomp : subcomponents) {
            arrayListSubComponent.add(subcomp);
        }

        // Distribute randomly!
        // First of all, give each of the clusters one element randomly
        for (int j = 0; j < clusters.size(); j++) {
            if (!arrayListSubComponent.isEmpty()) {
                int randN = randomNumberInRange(0, arrayListSubComponent.size() - 1);
                clusters.get(j).add(arrayListSubComponent.get(randN));
                arrayListSubComponent.remove(randN);
            }
        }

        int numberOfSubcomponents = arrayListSubComponent.size();
        // Then, a random element is assigned to a random cluster until every element is assigned
        for (int h = 0; h < numberOfSubcomponents; h++) {
            int randElement = randomNumberInRange(0, arrayListSubComponent.size() - 1);
            int randCluster = randomNumberInRange(0, clusters.size() - 1);

            clusters.get(randCluster).add(arrayListSubComponent.get(randElement));
            arrayListSubComponent.remove(randElement);
        }
        return clusters;
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