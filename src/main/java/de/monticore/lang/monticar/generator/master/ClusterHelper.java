package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMAPortBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.se_rwth.commons.logging.Log;
import org.jgrapht.Graph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

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
        clusters.forEach(c -> {
            if (c.size() == 1) {
                res.add(ExpandedComponentInstanceBuilder.clone(c.iterator().next()));
            } else {
                res.add(createECISFromCluster(c));
            }
        });
        return res;
    }

    private static ExpandedComponentInstanceSymbol createECISFromCluster(Set<ExpandedComponentInstanceSymbol> cluster) {
        //TODO: implement
        ExpandedComponentInstanceBuilder res = new ExpandedComponentInstanceBuilder();
        cluster.forEach(ecis -> {
            List<PortSymbol> unconnectedPorts = getUnconnectedPortsFromCluster(cluster);
            unconnectedPorts.forEach(p -> {
                PortSymbol superPort = EMAPortBuilder.clone(p);
                res.addPort(superPort);

            });

        });


        return res.build();
    }

    private static List<PortSymbol> getUnconnectedPortsFromCluster(Set<ExpandedComponentInstanceSymbol> cluster) {
        //TODO: implement
        return new ArrayList<>();
    }


}
