/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystemHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ComponentCall;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;

import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class RHSComponentInstanceSymbol extends EMAComponentInstanceSymbol {

    private final EMAEquationSystem equationSystem;
    private final SemiExplicitForm semiExplicitForm;

    public RHSComponentInstanceSymbol(EMAEquationSystem equationSystem) {
        super(equationSystem.getName() + "_RHS", null);
        this.equationSystem = equationSystem;
        this.semiExplicitForm = EMAEquationSystemHelper.buildSemiExplicitForm(equationSystem);
    }

    public EMAEquationSystem getEquationSystem() {
        return equationSystem;
    }

    public SemiExplicitForm getSemiExplicitForm() {
        return semiExplicitForm;
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getPortInstanceList() {
        Set<EMAPortInstanceSymbol> ports = new HashSet<>();
        ports.addAll(equationSystem.getIncomingPorts());
        ports.addAll(equationSystem.getOutgoingPorts());
        return ports;
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getIncomingPortInstances() {
        return equationSystem.getIncomingPorts();
    }

    @Override
    public Collection<EMAPortInstanceSymbol> getOutgoingPortInstances() {
        return equationSystem.getOutgoingPorts();
    }

    @Override
    public Collection<EMAComponentInstanceSymbol> getSubComponents() {
        Set<EMAComponentInstanceSymbol> components = semiExplicitForm.getG().stream()
                .filter(g -> g instanceof ComponentCall)
                .map(g -> ((ComponentCall) g).getComponent())
                .collect(Collectors.toSet());
//        return equationSystem.getComponentInstanceSymbols();
        return components;
    }

    @Override
    public Optional<EMAComponentInstanceSymbol> getSubComponent(String name) {
        return getSubComponents().stream().filter(s -> s.getName().equals(name) || s.getFullName().equals(name)).findAny();
    }

    @Override
    public Optional<EMAComponentInstanceSymbol> getParent() {
        return Optional.empty();
    }

    @Override
    public String toString() {
        return getName();
    }
}
