/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import java.util.Collection;
import java.util.List;

public class GraphHelper {
    public static void remove(EMAGraph graph, EMAVertex vertex) {
        graph.getVertices().remove(vertex);
        List<EMAEdge> edgesWithSourceNode = graph.getEdgesWithSourceNode(vertex);
        graph.getEdges().removeAll(edgesWithSourceNode);
        List<EMAEdge> edgesWithTargetNode = graph.getEdgeWithTargetNode(vertex);
        graph.getEdges().removeAll(edgesWithTargetNode);

        vertex.getOutports().stream().forEachOrdered(p -> graph.getPortMap().remove(p.getFullName()));
        vertex.getInports().stream().forEachOrdered(p -> graph.getPortMap().remove(p.getFullName()));
    }

    public static EMAGraph buildSubGraph(EMAGraph graph, Collection<EMAVertex> vertices) {
        EMAGraph subGraph = new EMAGraph();
        for (EMAVertex vertex : vertices) {
            subGraph.addVertex(vertex);
            for (EMAEdge emaEdge : graph.getEdgeWithTargetNode(vertex))
                if (vertices.contains(emaEdge.getSourceVertex()))
                    graph.addEdge(emaEdge);
            for (EMAEdge emaEdge : graph.getEdgesWithSourceNode(vertex))
                if (vertices.contains(emaEdge.getTargetVertex()))
                    graph.addEdge(emaEdge);
            for (EMAPort inport : vertex.getInports())
                subGraph.getPortMap().put(inport.getFullName(), inport);
            for (EMAPort outport : vertex.getOutports())
                subGraph.getPortMap().put(outport.getFullName(), outport);
        }
        return subGraph;
    }
}
