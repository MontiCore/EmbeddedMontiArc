/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

public class SListEntry {
    private final EMAComponentInstanceSymbol component;
    private final Call call;

    public SListEntry(EMAComponentInstanceSymbol component, Call call) {
        this.component = component;
        this.call = call;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }

    public Call getCall() {
        return call;
    }
}
