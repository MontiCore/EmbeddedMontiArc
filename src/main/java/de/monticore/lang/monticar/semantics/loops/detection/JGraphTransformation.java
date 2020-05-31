/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.monticar.semantics.loops.graph.EMAEdge;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.loops.graph.JGraphEdge;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;

public class JGraphTransformation {

    public Graph<EMAVertex, JGraphEdge> transform(EMAGraph emaGraph) {
        Graph<EMAVertex, JGraphEdge> graph = new DefaultDirectedGraph<EMAVertex, JGraphEdge>(JGraphEdge.class);

        for (EMAVertex vertex: emaGraph.getVertices()) {
            graph.addVertex(vertex);
        }

        for (EMAEdge edge: emaGraph.getEdges()) {
            graph.addEdge(edge.getSourceVertex(), edge.getTargetVertex());
        }

        return graph;
    }

}
