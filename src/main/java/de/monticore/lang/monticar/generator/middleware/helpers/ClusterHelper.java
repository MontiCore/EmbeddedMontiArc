package de.monticore.lang.monticar.generator.middleware.helpers;

import com.google.common.collect.Sets;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.se_rwth.commons.logging.Log;
import org.jgrapht.Graph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

import java.util.*;
import java.util.stream.Collectors;

public class ClusterHelper {

    private ClusterHelper() {
    }

    public static List<Set<ExpandedComponentInstanceSymbol>> getClusters(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        Graph<ExpandedComponentInstanceSymbol, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);

        componentInstanceSymbol.getSubComponents().forEach(graph::addVertex);
        graph.addVertex(componentInstanceSymbol);

        componentInstanceSymbol.getConnectors().stream()
                .filter(c -> !(c.getSourcePort().getMiddlewareSymbol().isPresent() && c.getTargetPort().getMiddlewareSymbol().isPresent()))
                .forEach(c -> {
                    ExpandedComponentInstanceSymbol compSource = c.getSourcePort().getComponentInstance().orElse(null);
                    ExpandedComponentInstanceSymbol compTarget = c.getTargetPort().getComponentInstance().orElse(null);
                    if (compSource == null || compTarget == null)
                        Log.error("ComponentInstance of source or target not found!");

                    graph.addEdge(compSource, compTarget);
                });

        ConnectivityInspector<ExpandedComponentInstanceSymbol, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        if (connectivityInspector.connectedSetOf(componentInstanceSymbol).size() != 1)
            Log.error("0x8EFC3: The supercomponent can only be connected to subcomponents via middleware ports!");

        graph.removeVertex(componentInstanceSymbol);

        return connectivityInspector.connectedSets();
    }

    public static List<ExpandedComponentInstanceSymbol> getClusterSubcomponents(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        List<Set<ExpandedComponentInstanceSymbol>> clusters = getClusters(componentInstanceSymbol);
        List<ExpandedComponentInstanceSymbol> res = new ArrayList<>();
        int[] i = {0};
        clusters.forEach(c -> {
            if (c.size() == 1) {
                res.add(ExpandedComponentInstanceBuilder.clone(c.iterator().next()));
            } else {
                //TODO: can lead to naming conflicts: subcomp that is named <componentName>Cluster<i[0]>
                String clusterName = componentInstanceSymbol.getName() + "Cluster" + i[0];
                i[0]++;
                res.add(createECISFromCluster(componentInstanceSymbol, c, clusterName));
            }
        });
        return res;
    }


    //    Cases: Connector Super,Sub -> Super,Sub
//    Super normal -> *: 0x8EFC3
//    Super Ros -> * normal: 0x8EFC3
//    Super Ros -> Super Ros: remove and warn
//    Super Ros -> Sub Ros: handled(let ros connect)
//    Sub normal -> * Ros: disallowed, RosTargetRosSender CoCo
//    Sub Ros -> Super normal: 0x8EFC3
//    Sub Ros -> Super Ros: handled(let ros connect)
//    Sub Ros -> Sub Ros: do nothing, wrapper and therefore the mw does not affect this level
//    Sub Ros -> Sub normal: handled(let comp connect)
    private static ExpandedComponentInstanceSymbol createECISFromCluster(ExpandedComponentInstanceSymbol inst, Set<ExpandedComponentInstanceSymbol> cluster, String clusterName) {
        //TODO: implement
        Set<PortSymbol> curClusterPorts = cluster.stream().flatMap(ecis -> ecis.getPorts().stream()).collect(Collectors.toSet());
        Set<PortSymbol> combinedPorts = new HashSet<>();
        combinedPorts.addAll(curClusterPorts);
        combinedPorts.addAll(inst.getPorts());

        Set<ConnectorSymbol> validConnectors = inst.getConnectors().stream()
                //remove all connections that use subcomponents not in cluster
                .filter(c -> combinedPorts.contains(c.getSourcePort()) && combinedPorts.contains(c.getTargetPort()))
                //remove all connections from super -> super and warn
                .filter(c -> {
                    if (inst.containsPort(c.getSourcePort()) && inst.containsPort(c.getTargetPort())) {
                        Log.warn("Direct connections from super comp to super comp are not supported!");
                        return false;
                    }
                    return true;
                })
                .collect(Collectors.toSet());

        Collection<PortSymbol> validPorts = validConnectors.stream()
                .flatMap(c -> Sets.newHashSet(c.getTargetPort(), c.getSourcePort()).stream())
                .filter(p -> !inst.containsPort(p))
                .collect(Collectors.toSet());

        return new ExpandedComponentInstanceBuilder()
                .setName(clusterName)
                .setSymbolReference(inst.getComponentType())
                .addPorts(validPorts)
                .addConnectors(validConnectors)
                .addSubComponents(cluster.stream().map(ExpandedComponentInstanceBuilder::clone).collect(Collectors.toList()))
                .addResolutionDeclarationSymbols(inst.getResolutionDeclarationSymbols())
                .build();
    }


}
