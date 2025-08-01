package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystemHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;

import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class EquationSystemComponentInstanceSymbol extends EMAComponentInstanceSymbol {

    private SemiExplicitForm semiExplicitForm;
    private EMAEquationSystem equationSystem;

    public EquationSystemComponentInstanceSymbol(EMAEquationSystem equationSystem) {
        super(equationSystem.getName(), null);
        this.equationSystem = equationSystem;
        this.semiExplicitForm = EMAEquationSystemHelper.buildSemiExplicitForm(equationSystem);
    }

    public EMAEquationSystem getEquationSystem() {
        return equationSystem;
    }

    public SemiExplicitForm getSemiExplicitForm() {
        return semiExplicitForm;
    }

    public String getSubComponentName(EMAComponentInstanceSymbol subComponent) {
        String parentName = NameHelper.getPackageOfFullQualifiedName(getFullName());
        String fullName = NameHelper.calculatePartialName(subComponent.getFullName(), parentName);
        return Joiners.DOT.join(getFullName(), fullName);
    }

    public VariableType getTypeOfSubComponent(EMAComponentInstanceSymbol subComponent) {
        String fullName = subComponent.getFullName();
        VariableType type = new VariableType();
        type.setTypeNameMontiCar(fullName);
        type.setTypeNameTargetLanguage(GeneralHelperMethods.getTargetLanguageComponentName(fullName));
        type.setIncludeName(type.getTypeNameTargetLanguage());
        TypeConverter.addNonPrimitiveVariableType(type);
        return type;
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
        return new HashSet<>();
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
