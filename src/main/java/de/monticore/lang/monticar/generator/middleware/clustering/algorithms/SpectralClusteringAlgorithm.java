package de.monticore.lang.monticar.generator.middleware.clustering.algorithms;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import smile.clustering.SpectralClustering;

import java.util.*;

// spectral clusterer product implementation
public class SpectralClusteringAlgorithm implements ClusteringAlgorithm {
    @Override
    public List<Set<ExpandedComponentInstanceSymbol>> cluster(ExpandedComponentInstanceSymbol component, int numClusters, Object... args) {

        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(component),
                labelsForSubcomps);

        SpectralClustering clustering;
        SpectralClusteringBuilder builder = new SpectralClusteringBuilder(adjMatrix, numClusters);

        // Handle optional additional params for SpectralClustering.
        // Additional params come as one or multiple key-value-pairs in the optional varargs array for this method,
        // with key as a string (containing the name of the parameter to pass thru to the spectral clusterer) followed by its value as an object
        String key;
        Object value;
        int v = 0;
        while (v < args.length) {
            if (args[v] instanceof String) {
                key = (String)args[v];
                if (v+1 < args.length) {
                    value = args[v + 1];
                    switch (key) {
                        case "l":
                            if (value instanceof Integer) {
                                builder.setL((Integer) value);
                            }
                            break;
                        case "sigma":
                            if (value instanceof Double) {
                                builder.setSigma((Double) value);
                            }
                            break;
                    }
                }
            }
            v = v + 2;
        }

        clustering = builder.build();
        //SpectralClustering clustering = new SpectralClustering(adjMatrix,numberOfClusters);

        int[] labels = clustering.getClusterLabel();

        List<Set<ExpandedComponentInstanceSymbol>> res = new ArrayList<>();

        for(int i = 0; i < clustering.getNumClusters(); i++){
            res.add(new HashSet<>());
        }

        subcompsOrderedByName.forEach(sc -> {
            int curClusterLabel = labels[labelsForSubcomps.get(sc.getFullName())];
            res.get(curClusterLabel).add(sc);
        });

        return res;
    }
}
