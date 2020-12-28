/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import org.jgrapht.graph.DefaultEdge;

public class JGraphEdge extends DefaultEdge {
    private EMAAtomicConnector ref;

    public EMAAtomicConnector getReferencedEdge() {
        return ref;
    }

    public void ReferencedEdge(EMAAtomicConnector ref) {
        this.ref = ref;
    }
}
