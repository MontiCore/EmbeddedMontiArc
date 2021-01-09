/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.JGraphEdge;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;

public class JGraphTransformation {

    public Graph<EMAComponentInstanceSymbol, JGraphEdge> transform(EMAGraph emaGraph) {
        Graph<EMAComponentInstanceSymbol, JGraphEdge> graph =
                new DefaultDirectedGraph<>(JGraphEdge.class);

        for (EMAComponentInstanceSymbol vertex: emaGraph.getVertices())
            graph.addVertex(vertex);

        for (EMAAtomicConnectorInstance edge: emaGraph.getEdges())
            graph.addEdge(edge.getSourceComponent(), edge.getTargetComponent());

        return graph;
    }

}
