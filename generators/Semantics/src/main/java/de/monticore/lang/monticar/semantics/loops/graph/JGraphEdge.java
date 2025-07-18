/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import org.jgrapht.graph.DefaultEdge;

public class JGraphEdge extends DefaultEdge {
    private EMAEdge ref;

    public EMAEdge getReferencedEdge() {
        return ref;
    }

    public void ReferencedEdge(EMAEdge ref) {
        this.ref = ref;
    }
}
