package de.monticore.lang.monticar.semantics.loops.graph;

public class EMAEdge {
    private EMAVertex sourceVertex;
    private EMAVertex targetVertex;
    private EMAPort sourcePort;
    private EMAPort targetPort;

    public EMAEdge(EMAVertex sourceVertex, EMAVertex targetVertex, EMAPort sourcePort, EMAPort targetPort) {
        this.sourceVertex = sourceVertex;
        this.targetVertex = targetVertex;
        this.sourcePort = sourcePort;
        this.targetPort = targetPort;
    }

    public EMAVertex getSourceVertex() {
        return sourceVertex;
    }

    public EMAVertex getTargetVertex() {
        return targetVertex;
    }

    public EMAPort getSourcePort() {
        return sourcePort;
    }

    public EMAPort getTargetPort() {
        return targetPort;
    }

    public String toString() {
        String sourcePortString = "";
        if (sourcePort != null)
            sourcePortString = "." + sourcePort.getName();
        String targetPortString = "";
        if (targetPort != null)
            targetPortString = "." + targetPort.getName();
        return "" + sourceVertex + sourcePortString + " -> " + targetVertex + targetPortString;
    }
}
