package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.se_rwth.commons.Joiners;

import java.util.Collection;
import java.util.Optional;

public class EquationSystemComponentInstanceSymbol extends EMAComponentInstanceSymbol {

    private EMAEquationSystem system;

    public EquationSystemComponentInstanceSymbol(EMAEquationSystem system) {
        super(system.getName(), null);
        this.system = system;
    }

    public String getSubComponentName(EMAComponentInstanceSymbol subComponent) {
        String parentName = NameHelper.getPackageOfFullQualifiedName(getFullName());
        String fullName = NameHelper.calculatePartialName(subComponent.getFullName(), parentName);
        return Joiners.DOT.join(getFullName(), fullName);
    }

    public VariableType getTypeOfSubComponent(EMAComponentInstanceSymbol subComponent) {
        String fullName = getSubComponentName(subComponent);
        VariableType type = new VariableType();
        type.setTypeNameMontiCar(fullName);
        type.setTypeNameTargetLanguage(GeneralHelperMethods.getTargetLanguageComponentName(fullName));
        type.setIncludeName(type.getTypeNameTargetLanguage());
        TypeConverter.addNonPrimitiveVariableType(type);
        return type;
    }

    @Override
    public Collection<EMAComponentInstanceSymbol> getSubComponents() {
        return system.getComponentInstanceSymbols();
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
