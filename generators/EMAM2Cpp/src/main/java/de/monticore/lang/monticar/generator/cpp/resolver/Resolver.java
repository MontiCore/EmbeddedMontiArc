/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.resolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.Scope;

import java.util.Optional;

public class Resolver {

    private Scope symTab;

    public Resolver(Scope symTab) {
        this.symTab = symTab;
    }

    /*
    public Optional<ComponentSymbol> getComponentSymbol(String component) {
        return symTab.resolve(component, ComponentSymbol.KIND);
    }

    public Optional<PortSymbol> getPortSymbol(String port) {
        return symTab.resolve(port, PortSymbol.KIND);
    }

    public Optional<ConnectorSymbol> getConnectorSymbol(String con) {
        return symTab.resolve(con, ConnectorSymbol.KIND);
    }

    public Optional<ComponentInstanceSymbol> getComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, ComponentInstanceSymbol.KIND);
    }
*/
    public Optional<EMAComponentInstanceSymbol> getExpandedComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, EMAComponentInstanceSymbol.KIND);
    }
}
