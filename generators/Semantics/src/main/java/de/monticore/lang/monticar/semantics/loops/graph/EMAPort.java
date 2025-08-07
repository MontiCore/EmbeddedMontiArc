package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

public class EMAPort {

    private String name;
    private String fullName;
    private EMAVertex emaVertex;
    private EMAPortInstanceSymbol referencedPort;

    public EMAPort(EMAPortInstanceSymbol referencedPort) {
        this.referencedPort = referencedPort;
        this.name = referencedPort.getName();
        this.fullName = referencedPort.getFullName();
    }

    public EMAPort(EMAPortInstanceSymbol referencedPort, String fullName, String name) {
        this.name = name;
        this.fullName = fullName;
        this.referencedPort = referencedPort;
    }

    public String getName() {
        return name;
    }

    public String getFullName() {
        return fullName;
    }

    public EMAPortInstanceSymbol getReferencedPort() {
        return referencedPort;
    }

    public String toString() {
        return fullName;
    }

    public EMAVertex getEmaVertex() {
        return emaVertex;
    }

    public void setEmaVertex(EMAVertex emaVertex) {
        this.emaVertex = emaVertex;
    }
}
