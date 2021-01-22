package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraphTransformation;
import de.monticore.lang.monticar.semantics.loops.graph.JGraphEdge;
import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.HawickJamesSimpleCycles;
import org.jgrapht.alg.interfaces.StrongConnectivityAlgorithm;

import java.util.*;
import java.util.stream.Collectors;

public class LoopDetection {

    /**
     * Detects loops in a component composition.
     * The result is a Set of Strongly Connected Components which manage a Set of Simple Cycles.
     * Artificial loops are detected as well, and are marked.
     */
    public static Set<StronglyConnectedComponent> detectLoops(EMAComponentInstanceSymbol component) {
        ConnectionHelper.resetCache();

        EMAGraph emaGraphNV = EMAGraphTransformation.transform(component, true, true);
        Set<StronglyConnectedComponent> stronglyConnectedComponentsNV = detectLoops(emaGraphNV);


        // Find artificial Loops


        EMAGraph emaGraph = EMAGraphTransformation.transform(component, true, false);
        Set<StronglyConnectedComponent> stronglyConnectedComponents = detectLoops(emaGraph);
        Set<SimpleCycle> simpleCycles = stronglyConnectedComponents
                .stream()
                .map(StronglyConnectedComponent::getSimpleCycles)
                .flatMap(Collection::stream)
                .collect(Collectors.toSet());

        for (StronglyConnectedComponent sccNV : stronglyConnectedComponentsNV) {
            // Filter out NV components. This means to only consider atomic components
            Set<EMAComponentInstanceSymbol> components = sccNV.getAllComponents()
                    .stream()
                    .filter(c -> c.isVirtual() || EMAPropertiesHelper.isAtomic(c))
                    .collect(Collectors.toSet());

            Optional<StronglyConnectedComponent> any = stronglyConnectedComponents
                    .stream()
                    .filter(scc -> scc.getAllComponents().containsAll(components))
                    .findAny();

            // If there is a loop, consisting only of atomic components, this is not an artificial loop
            if (!any.isPresent())
                sccNV.setArtificial(true);
            else {
                // Repeat for all simple cycles
                for (SimpleCycle simpleCycle : sccNV.getSimpleCycles()) {
                    // Filter out NV components. This means to only consider atomic components
                    Set<EMAComponentInstanceSymbol> simpleCycleComponents = simpleCycle.getAllComponents()
                            .stream()
                            .filter(c -> c.isVirtual() || EMAPropertiesHelper.isAtomic(c))
                            .collect(Collectors.toSet());

                    Optional<SimpleCycle> any1 = simpleCycles.
                            stream()
                            .filter(sc -> sc.getAllComponents().containsAll(simpleCycleComponents))
                            .findAny();

                    // If there is a loop, consisting only of atomic components, this is not an artificial loop
                    if (!any1.isPresent())
                        simpleCycle.setArtificial(true);
                }
            }
        }

        return stronglyConnectedComponentsNV;
    }

    public static Set<StronglyConnectedComponent> detectLoops(EMAGraph emaGraph) {
        JGraphTransformation jGraphTransformation = new JGraphTransformation();
        Graph<EMAComponentInstanceSymbol, JGraphEdge> graph = jGraphTransformation.transform(emaGraph);

        return detectLoops(graph, emaGraph);
    }

    public static Set<StronglyConnectedComponent> detectLoops(Graph<EMAComponentInstanceSymbol, JGraphEdge> graph,
                                                              EMAGraph emaGraph) {
        StrongConnectivityAlgorithm<EMAComponentInstanceSymbol, JGraphEdge> inspector = new KosarajuStrongConnectivityInspector<>(graph);
        List<Set<EMAComponentInstanceSymbol>> components = inspector.stronglyConnectedSets();

        Iterator<Set<EMAComponentInstanceSymbol>> iterator = components.iterator();
        while (iterator.hasNext()) {
            if (iterator.next().size() <= 1)
                iterator.remove();
        }

        HawickJamesSimpleCycles<EMAComponentInstanceSymbol, JGraphEdge> hawickJamesSimpleCycles =
                new HawickJamesSimpleCycles<>(graph);

        List<List<EMAComponentInstanceSymbol>> simpleCycles = hawickJamesSimpleCycles.findSimpleCycles();

        // Assign simple cycle to scc
        Set<StronglyConnectedComponent> res = new HashSet<>();
        for (Set<EMAComponentInstanceSymbol> stronglyConnectedComponent : components) {
            Set<List<EMAComponentInstanceSymbol>> simpleCyclesForStrongComponent = new HashSet<>();
            for (List<EMAComponentInstanceSymbol> simpleCycle : simpleCycles) {
                if (stronglyConnectedComponent.containsAll(simpleCycle))
                    simpleCyclesForStrongComponent.add(simpleCycle);
            }
            res.add(new StronglyConnectedComponent(stronglyConnectedComponent, simpleCyclesForStrongComponent, emaGraph));
        }
        return res;
    }
}
