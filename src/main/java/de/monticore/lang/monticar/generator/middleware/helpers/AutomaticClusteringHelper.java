package de.monticore.lang.monticar.generator.middleware.helpers;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.se_rwth.commons.logging.Log;
import smile.clustering.SpectralClustering;

import java.util.*;

public class AutomaticClusteringHelper {
    public static double[][] createAdjacencyMatrix(List<ExpandedComponentInstanceSymbol> subcomps, Collection<ConnectorSymbol> connectors, Map<String, Integer> subcompLabels) {
        // Nodes = subcomponents
        // Verts = connectors between subcomponents

        double[][] res = new double[subcomps.size()][subcomps.size()];

        connectors.forEach(con -> {
            Optional<ExpandedComponentInstanceSymbol> sourceCompOpt = con.getSourcePort().getComponentInstance();
            Optional<ExpandedComponentInstanceSymbol> targetCompOpt = con.getTargetPort().getComponentInstance();

            if (sourceCompOpt.isPresent() && targetCompOpt.isPresent()) {
                int index1 = subcompLabels.get(sourceCompOpt.get().getFullName());
                int index2 = subcompLabels.get(targetCompOpt.get().getFullName());

                res[index1][index2] = 1.0d;
                res[index2][index1] = 1.0d;
            } else {
                Log.error("0xADE65: Component of source or target not found!");
            }
        });


        return res;
    }

    public static List<Set<ExpandedComponentInstanceSymbol>> createClusters(ExpandedComponentInstanceSymbol component, int numberOfClusters, ClustererKind clustererKind){
        //TODO: create wrapper for clusterer for easy exchange

        List<ExpandedComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        double[][] adjMatrix = createAdjacencyMatrix(subcompsOrderedByName,
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

    public static void annotateComponentWithRosTagsForClusters(ExpandedComponentInstanceSymbol componentInstanceSymbol, List<Set<ExpandedComponentInstanceSymbol>> clusters) {
        Collection<ConnectorSymbol> connectors = componentInstanceSymbol.getConnectors();

        connectors.forEach(con -> {
            // -1 = super comp
            int sourceClusterLabel = -1;
            int targetClusterLabel = -1;

            ExpandedComponentInstanceSymbol sourceComp = con.getSourcePort().getComponentInstance().get();
            ExpandedComponentInstanceSymbol targetComp = con.getTargetPort().getComponentInstance().get();

            for(int i = 0; i < clusters.size(); i++){
                if(clusters.get(i).contains(sourceComp)){
                    sourceClusterLabel = i;
                }

                if(clusters.get(i).contains(targetComp)){
                    targetClusterLabel = i;
                }
            }

            if(sourceClusterLabel != targetClusterLabel){
                con.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol());
                con.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol());
            }

        });

    }
}
