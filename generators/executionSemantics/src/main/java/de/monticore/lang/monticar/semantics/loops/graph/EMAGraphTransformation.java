package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EMAGraphTransformation {


    public static EMAGraph transform(EMAComponentInstanceSymbol component,
                                     boolean considerDirectFeedThrough,
                                     boolean considerNonVirtual) {
        EMAGraph graph = new EMAGraph();

        // Add all subcomponents of this component, transitively
        Collection<EMAComponentInstanceSymbol> flattened = new HashSet<>();
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            flattened.addAll(getFlattened(subComponent, considerNonVirtual));
        flattened.stream().forEach(graph::addVertex);


        for (EMAComponentInstanceSymbol subComponent : graph.getVertices()) {
            // add only if component is df
            if (!considerDirectFeedThrough || subComponent.isDirectFeedThrough()) {
                // Adding for inports
                for (EMAPortInstanceSymbol inport : subComponent.getIncomingPortInstances()) {
                    Optional<EMAPortInstanceSymbol> source = ConnectionHelper.sourceOf(inport, considerNonVirtual);
                    if (!source.isPresent()) {
//                            Log.warn(String.format("TODO There is no incoming connection for port \"%s\"", inport.getFullName()));
                        continue;
                    }

                    // Add only inner graph connections
                    if (graph.getVertices().contains(source.get().getComponentInstance()))
                        graph.addEdge(new EMAAtomicConnectorInstance(source.get(), inport));
                }

//                // adding for outports, may be unnecessary, but it does not harm
//                for (EMAPortInstanceSymbol outport : subComponent.getComponentOutgoingPortInstances()) {
//                    Collection<EMAPortInstanceSymbol> targets = ConnectionHelper.targetsOf(outport, considerNonVirtual);
//                    for (EMAPortInstanceSymbol target : targets)
//                        // Add only inner graph connections
//                        if (graph.getVertices().contains(target.getComponentInstance()))
//                            graph.addEdge(new EMAAtomicConnectorInstance(outport, target));
//                }
            }
        }

        // if nonvirtual got considered, subcomponents of those are not part of the graph, add subgraphs here
        if (considerNonVirtual)
            getNonVirtual(component)
                    .stream()
                    .filter(c -> c != component)
                    .forEach(nv -> graph.addSubGraph(transform(nv, considerDirectFeedThrough, considerNonVirtual)));

        return graph;
    }

    private static Collection<EMAComponentInstanceSymbol> getFlattened(EMAComponentInstanceSymbol component, boolean considerNonVirtual) {
        if (EMAPropertiesHelper.isAtomic(component))
            return Collections.singleton(component);
        if (considerNonVirtual && EMAPropertiesHelper.isNonVirtual(component))
            return Collections.singleton(component);

        return component.getSubComponents().stream()
                .map(s -> getFlattened(s, considerNonVirtual))
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());
    }

    private static Collection<EMAComponentInstanceSymbol> getNonVirtual(EMAComponentInstanceSymbol component) {
        Set result = new HashSet();
        if (EMAPropertiesHelper.isNonVirtual(component))
            result.add(component);

        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(getNonVirtual(subComponent));
        return result;
    }

}
