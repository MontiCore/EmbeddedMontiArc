/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.detection.Detection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.loops.graph.GraphHelper;

import java.util.Set;
import java.util.stream.Collectors;

public class ConnectedComponentHelper {
    public static void remove(ConnectedComponent connectedComponent, EMAVertex emaVertex) {
        connectedComponent.getAllComponents().remove(emaVertex);
        Set<EMAPort> inportsToRemove = connectedComponent.getInports()
                .stream()
                .filter(i -> i.getEmaVertex().equals(emaVertex))
                .collect(Collectors.toSet());
        Set<EMAPort> outportsToRemove = connectedComponent.getOutports()
                .stream()
                .filter(i -> i.getEmaVertex().equals(emaVertex))
                .collect(Collectors.toSet());
        connectedComponent.getInports().removeAll(inportsToRemove);
        connectedComponent.getOutports().removeAll(outportsToRemove);
        Set<EMAPort> portStatementsToRemove = connectedComponent.getPortStatements().entrySet()
                .stream()
                .filter(ps -> inportsToRemove.contains(ps.getKey()) || outportsToRemove.contains(ps.getKey()))
                .map(ps -> ps.getKey())
                .collect(Collectors.toSet());
        portStatementsToRemove.stream().forEachOrdered(ps -> connectedComponent.getPortStatements().remove(ps));

        if (connectedComponent instanceof StrongConnectedComponent)
            for (SimpleCycle simpleCycle : ((StrongConnectedComponent) connectedComponent).getSimpleCycles())
                remove(simpleCycle, emaVertex);
    }

    public static void updateCycles(ConnectedComponent connectedComponent) {
        // TODO
        EMAGraph emaGraph = GraphHelper.buildSubGraph(connectedComponent.getGraph(), connectedComponent.getAllComponents());
        Set<StrongConnectedComponent> strongConnectedComponents = Detection.detectLoops(emaGraph);
        if (connectedComponent instanceof StrongConnectedComponent) {
            Set<StrongConnectedComponent> collect = strongConnectedComponents.stream()
                    .filter(c -> c.getAllComponents().equals(connectedComponent.getAllComponents()))
                    .collect(Collectors.toSet());
//            if (collect.size() == 1)
        }
    }
}
