package de.monticore.lang.monticar.generator.middleware.clustering.algorithms;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import smile.clustering.KMeans;

import java.util.*;

public class KMeansAlgorithm implements ClusteringAlgorithm {

    public static final int MAX_ITER = 10;

    @Override
    public List<Set<ExpandedComponentInstanceSymbol>> cluster(ExpandedComponentInstanceSymbol component, int numberOfClusters) {
        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(component),
                labelsForSubcomps);

        KMeans clustering = new KMeans(adjMatrix,numberOfClusters, MAX_ITER);

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
