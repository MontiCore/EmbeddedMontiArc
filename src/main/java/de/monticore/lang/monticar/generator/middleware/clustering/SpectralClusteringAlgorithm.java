package de.monticore.lang.monticar.generator.middleware.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import smile.clustering.SpectralClustering;

import java.util.*;

// spectral clusterer product implementation
public class SpectralClusteringAlgorithm implements ClusteringAlgorithm {
    @Override
    public List<Set<ExpandedComponentInstanceSymbol>> cluster(ExpandedComponentInstanceSymbol component, int numberOfClusters) {

        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(component),
                labelsForSubcomps);

        SpectralClustering clustering = new SpectralClustering(adjMatrix,numberOfClusters);

        int[] labels = clustering.getClusterLabel();

        List<Set<ExpandedComponentInstanceSymbol>> res = new ArrayList<>();

        for(int i = 0; i < numberOfClusters; i++){
            res.add(new HashSet<>());
        }

        subcompsOrderedByName.forEach(sc -> {
            int curClusterLabel = labels[labelsForSubcomps.get(sc.getFullName())];
            res.get(curClusterLabel).add(sc);
        });

        return res;
    }
}
