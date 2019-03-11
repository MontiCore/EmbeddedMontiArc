package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringBuilder;

import java.util.*;

public class MonteCarloIntegration {


    // index 1: Spectral Clustering
    // Index 2: Random Clustering
    public static double simulate(int iterations, EMAComponentInstanceSymbol componentInstanceSymbol, int numberOfClusters) {
        //EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        double sum = 0;

        double[] costs = new double[iterations];

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
            System.out.println("Average(" + (i + 1) + ")=" + sum);
            /*
            if(i==iterations-1){
                System.out.println("Costlist: "+ Arrays.toString(costs));
            }
            */
        }

        System.out.println("Final Result: " + sum);
        System.out.println("Minimum: "+ getMinValue(costs));
        System.out.println("Maximum: "+ getMaxValue(costs));
        // Plot(x,y) would be Plot(iterations, getCostAtN(i+1
        return sum;
    }

    public static int randomNumberInRange(int min, int max) {
        Random random = new Random();
        return random.nextInt((max - min) + 1) + min;
    }

    // getting the maximum value
    public static double getMaxValue(double[] array) {
        double maxValue = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > maxValue) {
                maxValue = array[i];
            }
        }
        return maxValue;
    }

    // getting the miniumum value
    public static double getMinValue(double[] array) {
        double minValue = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] < minValue) {
                minValue = array[i];
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
    public static double getCostAtN(int n, double[] costs) {
        double sum = 0;

        for (int i = 0; i < n ; i++) {
            sum += costs[i];
        }

        return sum /n;
    }
}