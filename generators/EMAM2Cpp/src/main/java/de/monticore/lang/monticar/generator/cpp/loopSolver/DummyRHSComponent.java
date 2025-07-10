/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.se_rwth.commons.Names;

import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class DummyRHSComponent extends EMAComponentInstanceSymbol {

    private final EMAComponentInstanceSymbol originalSymbol;

    public DummyRHSComponent(EMAComponentInstanceSymbol originalSymbol) {
        super(originalSymbol.getName() + "_RHS", null);
        this.originalSymbol = originalSymbol;
        this.setPackageName(originalSymbol.getPackageName());
        this.setFullName(Names.getQualifiedName(getPackageName(), getName()));
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getPortInstanceList() {
        return originalSymbol.getIncomingPortInstances();
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getIncomingPortInstances() {
        return originalSymbol.getIncomingPortInstances();
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getOutgoingPortInstances() {
        return new HashSet<>();
    }

    @Override
    public Collection<EMAComponentInstanceSymbol> getSubComponents() {
        return new HashSet<>();
    }

    @Override
    public Optional<EMAComponentInstanceSymbol> getSubComponent(String name) {
        return Optional.empty();
    }

    @Override
    public Optional<EMAComponentInstanceSymbol> getParent() {
        return Optional.empty();
    }

    @Override
    public String toString() {
        return getFullName();
    }
}
