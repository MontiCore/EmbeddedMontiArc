/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

public class EMAAtomicConnector {
    private EMAPortInstanceSymbol source;
    private EMAPortInstanceSymbol target;

    public EMAAtomicConnector(EMAPortInstanceSymbol source, EMAPortInstanceSymbol target) {
        this.source = source;
        this.target = target;
    }

    public EMAPortInstanceSymbol getSourcePort() {
        return source;
    }

    public EMAComponentInstanceSymbol getSourceComponent() {
        return getSourcePort().getComponentInstance();
    }

    public void setSource(EMAPortInstanceSymbol source) {
        this.source = source;
    }

    public EMAPortInstanceSymbol getTargetPort() {
        return target;
    }

    public EMAComponentInstanceSymbol getTargetComponent() {
        return getTargetPort().getComponentInstance();
    }

    public void setTarget(EMAPortInstanceSymbol target) {
        this.target = target;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof EMAAtomicConnector)) return false;
        if (!source.equals(((EMAAtomicConnector) obj).getSourcePort())) return false;
        if (!target.equals(((EMAAtomicConnector) obj).getTargetPort())) return false;
        return true;
    }
}
