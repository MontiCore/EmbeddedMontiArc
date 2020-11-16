/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

public class EMAAtomicConnector {
    private EMAPortInstanceSymbol source;
    private EMAPortInstanceSymbol target;

    public EMAAtomicConnector(EMAPortInstanceSymbol source, EMAPortInstanceSymbol target) {
        this.source = source;
        this.target = target;
    }

    public EMAPortInstanceSymbol getSource() {
        return source;
    }

    public void setSource(EMAPortInstanceSymbol source) {
        this.source = source;
    }

    public EMAPortInstanceSymbol getTarget() {
        return target;
    }

    public void setTarget(EMAPortInstanceSymbol target) {
        this.target = target;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof EMAAtomicConnector)) return false;
        if (!source.equals(((EMAAtomicConnector) obj).getSource())) return false;
        if (!target.equals(((EMAAtomicConnector) obj).getTarget())) return false;
        return true;
    }
}
