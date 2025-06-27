/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

public class ComponentCall implements EquationSystemFunction {

    private final EMAPortInstanceSymbol port;
    private final EMAComponentInstanceSymbol component;

    public ComponentCall(EMAComponentInstanceSymbol component, EMAPortInstanceSymbol port) {
        this.port = port;
        this.component = component;
    }

    public EMAPortInstanceSymbol getPort() {
        return port;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }
}
