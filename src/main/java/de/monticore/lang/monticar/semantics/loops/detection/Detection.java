package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.loops.graph.JGraphEdge;
import org.jgrapht.Graph;
import org.jgrapht.alg.connectivity.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.HawickJamesSimpleCycles;
import org.jgrapht.alg.interfaces.StrongConnectivityAlgorithm;

import java.util.*;

public class Detection {

    public Loops detectLoops(EMAComponentSymbol component) {
        EMAGraphTransformation emaTransformation = new EMAGraphTransformation();
        EMAGraph emaGraph = emaTransformation.transform(component);

        JGraphTransformation jGraphTransformation = new JGraphTransformation();
        Graph<EMAVertex, JGraphEdge> graph = jGraphTransformation.transform(emaGraph);

        StrongConnectivityAlgorithm<EMAVertex, JGraphEdge> inspector = new KosarajuStrongConnectivityInspector<>(graph);
        List<Set<EMAVertex>> components = inspector.stronglyConnectedSets();

        Iterator<Set<EMAVertex>> iterator = components.iterator();
        while (iterator.hasNext()) {
            if (iterator.next().size() <= 1)
                iterator.remove();
        }

        HawickJamesSimpleCycles<EMAVertex, JGraphEdge> hawickJamesSimpleCycles​ =
                new HawickJamesSimpleCycles<>(graph);

        List<List<EMAVertex>> simpleCycles = hawickJamesSimpleCycles​.findSimpleCycles();

        return new Loops(simpleCycles, components, emaGraph);
    }

    public class Loops {

        public Loops(List<List<EMAVertex>> simpleCycles, List<Set<EMAVertex>> components, EMAGraph graph) {
            this.simpleCycles = simpleCycles;
            this.components = components;
            this.graph = graph;

            this.cyclesForComponent = new HashMap<>();

            for (Set<EMAVertex> component : this.components) {
                this.cyclesForComponent.put(component, new LinkedList<>());
                for (List<EMAVertex> cycle : this.simpleCycles) {
                    if (component.containsAll(cycle)) {
                        this.cyclesForComponent.get(component).add(cycle);
                    }
                }
            }
        }


        private final List<List<EMAVertex>> simpleCycles;
        private final List<Set<EMAVertex>> components;
        private final Map<Set<EMAVertex>, List<List<EMAVertex>>> cyclesForComponent;

        private final EMAGraph graph;

        public List<List<EMAVertex>> getSimpleCycles() {
            return simpleCycles;
        }

        public List<Set<EMAVertex>> getComponents() {
            return components;
        }

        public Map<Set<EMAVertex>, List<List<EMAVertex>>> getCyclesForComponent() {
            return cyclesForComponent;
        }

        public EMAGraph getGraph() {
            return graph;
        }
    }
}
