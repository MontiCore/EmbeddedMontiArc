/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

public class EMAPortEdge extends EMAEdge {

    public EMAPortEdge(EMAVertex sourceVertex, EMAVertex targetVertex, EMAPort sourcePort, EMAPort targetPort) {
        super(sourceVertex, targetVertex, sourcePort, targetPort);
    }

    public String toString() {
        return "" + getSourceVertex() + " -> " + getTargetVertex();
    }
}
