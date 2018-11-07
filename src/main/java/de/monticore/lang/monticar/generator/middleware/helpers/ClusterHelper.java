package de.monticore.lang.monticar.generator.middleware.helpers;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;
import org.jgrapht.Graph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;
import smile.clustering.SpectralClustering;

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
                    if (compSource == null || compTarget == null) {
                        Log.error("ComponentInstance of source or target not found!");
                    }
                    if (!compSource.equals(compTarget)) {
                        graph.addEdge(compSource, compTarget);
                    }
                });

        ConnectivityInspector<ExpandedComponentInstanceSymbol, DefaultEdge> connectivityInspector = new ConnectivityInspector<>(graph);
        int instanceSetSize = connectivityInspector.connectedSetOf(componentInstanceSymbol).size();
        if (instanceSetSize != 1) {
            Log.warn("0x8EFC3: The supercomponent can only be connected to subcomponents via middleware ports!");
            return new ArrayList<>();
        }
        //All subcomps are connected via non mw -> only highest level needs to be generated
        if (instanceSetSize != componentInstanceSymbol.getSubComponents().size() + 1)
            graph.removeVertex(componentInstanceSymbol);

        List<Set<ExpandedComponentInstanceSymbol>> res = connectivityInspector.connectedSets();
        return res;
    }

    public static List<ExpandedComponentInstanceSymbol> getClusterSubcomponents(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        List<Set<ExpandedComponentInstanceSymbol>> clusters = getClusters(componentInstanceSymbol);
        List<ExpandedComponentInstanceSymbol> res = new ArrayList<>();
        int[] i = {0};
        clusters.forEach(c -> {
            if (c.size() == 1) {
                res.add(c.iterator().next());
            } else {
                String clusterPrefix = componentInstanceSymbol.getName() + "_cluster";
                while (componentInstanceSymbol.getSubComponent(clusterPrefix + i[0]).isPresent()) {
                    i[0]++;
                }
                res.add(createECISFromCluster(componentInstanceSymbol, c, clusterPrefix + i[0]));
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
        Set<PortSymbol> curClusterPorts = cluster.stream().flatMap(ecis -> ecis.getPortsList().stream()).collect(Collectors.toSet());
        Set<PortSymbol> combinedPorts = new HashSet<>();
        combinedPorts.addAll(curClusterPorts);
        combinedPorts.addAll(inst.getPortsList());

        Set<ConnectorSymbol> tmpConnectiors = inst.getConnectors().stream()
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
                //remove all connections super <-> cluster
                .filter(c -> !(inst.getPortsList().contains(c.getSourcePort()) || (inst.getPortsList().contains(c.getTargetPort()))))
                .collect(Collectors.toSet());

        Collection<PortSymbol> mwPorts = curClusterPorts.stream()
                .filter(p -> p.getMiddlewareSymbol().isPresent())
                .collect(Collectors.toSet());

        List<PortSymbol> tmpPorts = mwPorts.stream()
                .filter(p -> p.getMiddlewareSymbol().isPresent())
                .map(p -> {
                    String sourcePortName;
                    String targetPortName;
                    String subName = p.getComponentInstance().get().getName();
                    if (p.isIncoming()) {
                        sourcePortName = p.getName();
                        targetPortName = subName + "." + p.getName();
                    } else {
                        sourcePortName = subName + "." + p.getName();
                        targetPortName = p.getName();
                    }
                    ConnectorSymbol tmpConnector = ConnectorSymbol.builder()
                            .setSource(sourcePortName)
                            .setTarget(targetPortName)
                            .build();
                    tmpConnectiors.add(tmpConnector);
                    return EMAPortBuilder.clone(p);
                })
                .collect(Collectors.toList());

        Set<ResolvingFilter<? extends Symbol>> resolvingFilters = inst.getSpannedScope().getResolvingFilters();
        List<ExpandedComponentInstanceSymbol> tmpSubcomps = cluster.stream().map(ExpandedComponentInstanceBuilder::clone).collect(Collectors.toList());
        tmpSubcomps.forEach(sc -> ((CommonScope) sc.getSpannedScope()).setResolvingFilters(resolvingFilters));
        ExpandedComponentInstanceSymbol res = new ExpandedComponentInstanceBuilder()
                .setName(clusterName)
                .setSymbolReference(inst.getComponentType())
                .addPorts(tmpPorts)
                .addConnectors(tmpConnectiors)
                .addSubComponents(tmpSubcomps)
                .addResolutionDeclarationSymbols(inst.getResolutionDeclarationSymbols())
                .build();

        ((CommonScope) res.getSpannedScope()).setResolvingFilters(resolvingFilters);
        res.setEnclosingScope((MutableScope) inst.getEnclosingScope());
        return res;
    }


    public static List<ExpandedComponentInstanceSymbol> getSubcompsOrderedByName(ExpandedComponentInstanceSymbol componentInstanceSymbol){
        return componentInstanceSymbol.getSubComponents().stream()
                .sorted(Comparator.comparing(ExpandedComponentInstanceSymbol::getFullName))
                .collect(Collectors.toList());
    }

    public static Collection<ConnectorSymbol> getInnerConnectors(ExpandedComponentInstanceSymbol componentInstanceSymbol){
        String superCompName = componentInstanceSymbol.getFullName();
        return componentInstanceSymbol.getConnectors().stream()
                //filter out all connectors to super component
                .filter(con -> !con.getSourcePort().getComponentInstance().get().getFullName().equals(superCompName)
                        && !con.getTargetPort().getComponentInstance().get().getFullName().equals(superCompName))
                .collect(Collectors.toList());
    }


    public static Map<String, Integer> getLabelsForSubcomps(List<ExpandedComponentInstanceSymbol> subcomps) {
        Map<String, Integer> componentIndecies = new HashMap<>();

        int[] i = {0};
        subcomps.forEach(sc -> componentIndecies.put(sc.getFullName(), i[0]++));
        return componentIndecies;
    }

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

    public static List<ConnectorSymbol> findConnectorsForRosTagging(int[] clusterlabels, ExpandedComponentInstanceSymbol component){
        Collection<ExpandedComponentInstanceSymbol> subcomps = component.getSubComponents().stream()
                .sorted(Comparator.comparing(ExpandedComponentInstanceSymbol::getFullName))
                .collect(Collectors.toList());

        Map<String, Integer> componentIndecies = new HashMap<>();

        String superCompName = component.getFullName();
        int[] i = {0};
        subcomps.forEach(sc -> componentIndecies.put(sc.getFullName(), i[0]++));

        Collection<ConnectorSymbol> connectors = component.getConnectors().stream()
                //filter out all connectors to super component
                .filter(con -> !con.getSourcePort().getComponentInstance().get().getFullName().equals(superCompName)
                        && !con.getTargetPort().getComponentInstance().get().getFullName().equals(superCompName))
                .collect(Collectors.toList());

        List<ConnectorSymbol> res = new LinkedList<>();

        connectors.forEach(con -> {
            Optional<ExpandedComponentInstanceSymbol> sourceCompOpt = con.getSourcePort().getComponentInstance();
            Optional<ExpandedComponentInstanceSymbol> targetCompOpt = con.getTargetPort().getComponentInstance();

            if (sourceCompOpt.isPresent() && targetCompOpt.isPresent()) {
                if (clusterlabels[componentIndecies.get(sourceCompOpt.get().getFullName())] !=
                        clusterlabels[componentIndecies.get(targetCompOpt.get().getFullName())]){
                    res.add(con);
                }
            } else {
                Log.error("0xADE65: Component of source or target not found!");
            }
        });

        return res;
    }

}
