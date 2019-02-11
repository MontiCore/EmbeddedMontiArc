package de.monticore.lang.monticar.generator.middleware.Simulation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.SpectralClusteringBuilder;

import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;

public class MonteCarloIntegration {

    public double simulate(int iterations, EMAComponentInstanceSymbol componentInstanceSymbol){
        EMAComponentInstanceSymbol flattenedComponent = FlattenArchitecture.flattenArchitecture(componentInstanceSymbol);

        double sum = 0;

        for(long i = 0; i<iterations; i++){
            // Cluster with Spectral + save the cost: Parameter(Number of clusters, data)
            int randNumClusters = randomNumberInRange(1, componentInstanceSymbol.getSubComponents().size());

            SpectralClusteringAlgorithm spectralClusteringAlgorithm = new SpectralClusteringAlgorithm();
            Object[] params = new Object[]{SpectralClusteringBuilder.SpectralParameters.SPECTRAL_NUM_CLUSTERS, randNumClusters};
            List<Set<EMAComponentInstanceSymbol>> spectralClusters = spectralClusteringAlgorithm.cluster(flattenedComponent, params);
            AutomaticClusteringHelper.annotateComponentWithRosTagsForClusters(flattenedComponent, spectralClusters);

            //iterate through all clusters and add all cost of the ROS Tags between clusters
            sum+=calculateCostOfClusters(componentInstanceSymbol, spectralClusters);
        }

        return sum/iterations;
    }

    public int randomNumberInRange(int min, int max) {
        Random random = new Random();
        return random.nextInt((max - min) + 1) + min;
    }

    public static double calculateCostOfClusters(EMAComponentInstanceSymbol componentInstanceSymbol, List<Set<EMAComponentInstanceSymbol>> clusters) {
        Collection<EMAConnectorInstanceSymbol> connectors = componentInstanceSymbol.getConnectorInstances();

        double sum = 0;

        for(EMAConnectorInstanceSymbol con : connectors){
            // -1 = super comp
            int sourceClusterLabel = -1;
            int targetClusterLabel = -1;

            EMAComponentInstanceSymbol sourceComp = con.getSourcePort().getComponentInstance();
            EMAComponentInstanceSymbol targetComp = con.getTargetPort().getComponentInstance();

            for(int i = 0; i < clusters.size(); i++){
                if(clusters.get(i).contains(sourceComp)){
                    sourceClusterLabel = i;
                }

                if(clusters.get(i).contains(targetComp)){
                    targetClusterLabel = i;
                }
            }
            if(sourceClusterLabel != targetClusterLabel){
                sum +=AutomaticClusteringHelper.getTypeCostHeuristic(con.getSourcePort());
            }

        }

        return sum;
    }
}
