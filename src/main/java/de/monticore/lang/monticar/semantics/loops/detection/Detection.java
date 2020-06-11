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

    public Set<StrongConnectedComponent> detectLoops(EMAComponentSymbol component) {
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
