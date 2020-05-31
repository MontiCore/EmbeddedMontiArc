package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;

public class EMAPort {

    private String name;
    private String fullName;
    private EMAPortSymbol referencedPort;

    public EMAPort(String name, String fullName, EMAPortSymbol referencedPort) {
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

    public EMAPortSymbol getReferencedPort() {
        return referencedPort;
    }

    public String toString() {
        return fullName;
    }
}
