package de.monticore.lang.monticar.generator.cpp.resolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.montiarc.montiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.symboltable.Scope;

import java.util.Optional;

public class Resolver {

    private Scope symTab;

    public Resolver(Scope symTab) {
        this.symTab = symTab;
    }

    public Optional<EMAComponentSymbol> getComponentSymbol(String component) {
        return symTab.resolve(component, EMAComponentSymbol.KIND);
    }

    public Optional<EMAPortInstanceSymbol> getEMAPortInstanceSymbol(String port) {
        return symTab.resolve(port, EMAPortInstanceSymbol.KIND);
    }

    public Optional<EMAConnectorSymbol> getEMAConnectorSymbol(String con) {
        return symTab.resolve(con, EMAConnectorSymbol.KIND);
    }

    public Optional<ComponentInstanceSymbol> getComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, ComponentInstanceSymbol.KIND);
    }

    public Optional<EMAComponentInstanceSymbol> getEMAComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, EMAComponentInstanceSymbol.KIND);
    }
}
