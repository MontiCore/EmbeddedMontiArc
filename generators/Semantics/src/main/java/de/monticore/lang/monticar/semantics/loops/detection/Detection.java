package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.loops.graph.JGraphEdge;
import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.HawickJamesSimpleCycles;
import org.jgrapht.alg.interfaces.StrongConnectivityAlgorithm;

import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class Detection {

    public static Set<StrongConnectedComponent> detectLoops(EMAComponentInstanceSymbol component) {
        EMAGraphTransformation emaTransformation = new EMAGraphTransformation();
        EMAGraph emaGraph = emaTransformation.transform(component);

        return detectLoops(emaGraph);
    }

    public static Set<StrongConnectedComponent> detectLoops(EMAGraph emaGraph) {
        JGraphTransformation jGraphTransformation = new JGraphTransformation();
        Graph<EMAVertex, JGraphEdge> graph = jGraphTransformation.transform(emaGraph);

        return detectLoops(graph, emaGraph);
    }

    public static Set<StrongConnectedComponent> detectLoops(Graph<EMAVertex, JGraphEdge> graph, EMAGraph emaGraph) {
        StrongConnectivityAlgorithm<EMAVertex, JGraphEdge> inspector = new KosarajuStrongConnectivityInspector<>(graph);
        List<Set<EMAVertex>> components = inspector.stronglyConnectedSets();

        Iterator<Set<EMAVertex>> iterator = components.iterator();
        while (iterator.hasNext()) {
            if (iterator.next().size() <= 1)
                iterator.remove();
        }

        HawickJamesSimpleCycles<EMAVertex, JGraphEdge> hawickJamesSimpleCycles =
                new HawickJamesSimpleCycles<>(graph);

        List<List<EMAVertex>> simpleCycles = hawickJamesSimpleCycles.findSimpleCycles();

        Set<StrongConnectedComponent> res = new HashSet<>();
        for (Set<EMAVertex> strongConnectedComponent : components) {
            Set<List<EMAVertex>> simpleCyclesForStrongComponent = new HashSet<>();
            for (List<EMAVertex> simpleCycle : simpleCycles) {
                if (strongConnectedComponent.containsAll(simpleCycle))
                    simpleCyclesForStrongComponent.add(simpleCycle);
            }
            res.add(new StrongConnectedComponent(strongConnectedComponent, simpleCyclesForStrongComponent, emaGraph));
        }
        return res;
    }
}
