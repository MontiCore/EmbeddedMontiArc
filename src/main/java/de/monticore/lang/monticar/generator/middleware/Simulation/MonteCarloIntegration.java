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
    public static double simulate(int iterations, EMAComponentInstanceSymbol componentInstanceSymbol, int numberOfClusters, int index){
        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        double sum = 0;

        if(index == 1){
            for(int i = 0; i<iterations; i++){
                // This would be with Spectral Clustering
                SpectralClusteringAlgorithm spectralClusteringAlgorithm = new SpectralClusteringAlgorithm();
                Object[] params = new Object[]{SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS, numberOfClusters};
                List<Set<EMAComponentInstanceSymbol>> clusters = spectralClusteringAlgorithm.cluster(flattenedComponent, params);

                //iterate through all clusters and add all cost of the ROS Tags between clusters
                sum += AutomaticClusteringHelper.getTypeCostHeuristic(flattenedComponent, clusters);
            }
        }
        else if(index == 2) {
            for (int j = 0; j < iterations; j++) {
                // Let's Random cluster the model
                List<Set<EMAComponentInstanceSymbol>> clusters = randomClustering(flattenedComponent, numberOfClusters);

                //iterate through all clusters and add all cost of the ROS Tags between clusters
                sum += AutomaticClusteringHelper.getTypeCostHeuristic(flattenedComponent, clusters);
            }
        }
        // return average costs of clustering with spectral
        double res = sum/iterations;
        System.out.println("Average Costs: " + res);
        return res;
    }

    public static int randomNumberInRange(int min, int max) {
        Random random = new Random();
        return random.nextInt((max - min) + 1) + min;
    }

    public static List<Set<EMAComponentInstanceSymbol>> randomClustering(EMAComponentInstanceSymbol componentInstanceSymbol, int numberOfClusters){
        List<Set<EMAComponentInstanceSymbol>> clusters = new ArrayList<>();

        for(int i = 0; i<numberOfClusters; i++){
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
        for(int j = 0; j < clusters.size(); j++){
            if(!arrayListSubComponent.isEmpty()) {
                int randN = randomNumberInRange(0, arrayListSubComponent.size()-1);
                clusters.get(j).add(arrayListSubComponent.get(randN));
                arrayListSubComponent.remove(randN);
            }
        }

        int numberOfSubcomponents = arrayListSubComponent.size();
        // Then, a random element is assigned to a random cluster until every element is assigned
        for(int h = 0; h<numberOfSubcomponents; h++){
            int randElement = randomNumberInRange(0,arrayListSubComponent.size()-1);
            int randCluster = randomNumberInRange(0,clusters.size()-1);

            clusters.get(randCluster).add(arrayListSubComponent.get(randElement));
            arrayListSubComponent.remove(randElement);
        }

        return clusters;
    }
}