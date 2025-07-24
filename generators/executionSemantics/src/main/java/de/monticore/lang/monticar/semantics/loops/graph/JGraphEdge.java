/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import org.jgrapht.graph.DefaultEdge;

public class JGraphEdge extends DefaultEdge {
    private EMAAtomicConnectorInstance ref;

    public EMAAtomicConnectorInstance getReferencedEdge() {
        return ref;
    }

    public void ReferencedEdge(EMAAtomicConnectorInstance ref) {
        this.ref = ref;
    }
}
