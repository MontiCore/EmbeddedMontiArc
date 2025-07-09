/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.AnalyzeEquationSystemType;
import de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystemHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.monticore.lang.monticar.semantics.resolve.SymbolTableHelper;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;

import java.util.*;

public class CPPEquationSystemHelper {

    public static String getNameOfPortOfComponent(EMAPortInstanceSymbol port) {
        String componentName = getNameOfComponent(port.getComponentInstance());
        return String.join(".", componentName, port.getName());
    }

    public static String getNameOfPort(EMAPortInstanceSymbol port) {
        return NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(port));
    }

    public static String getNameOfComponent(EMAComponentInstanceSymbol component) {
        return NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(component));
    }

    public static void handleEquationSystem(SemiExplicitForm semiExplicitForm,
                                             EMAMBluePrintCPP bluePrint,
                                             GeneratorCPP generatorCPP,
                                             List<String> includeStrings) {
        EquationSystemType type = AnalyzeEquationSystemType.typeOf(semiExplicitForm);

        switch (type) {
            case Linear:
            case NonLinear:
            case Polynom:
                NumericSolvers.getNonLinearSolveGenerator().handleEquationSystem(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
            case ODE:
                NumericSolvers.getODESolveGenerator().handleEquationSystem(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
            case DAE:
                NumericSolvers.getDAESolveGenerator().handleEquationSystem(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
        }
    }

    public static void handleRHS(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        EquationSystemType type = AnalyzeEquationSystemType.typeOf(semiExplicitForm);

        switch (type) {
            case Linear:
            case NonLinear:
            case Polynom:
                NumericSolvers.getNonLinearSolveGenerator().handleRHS(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
            case ODE:
                NumericSolvers.getODESolveGenerator().handleRHS(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
            case DAE:
                NumericSolvers.getDAESolveGenerator().handleRHS(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
                break;
        }
    }

    public static void handleSubComponents(EMAEquationSystem equationSystem) {
        for (EMAComponentInstanceSymbol subComponent : equationSystem.getComponentInstanceSymbols()) {
            String name = subComponent.getName();
            String packageName = getSubComponentName(equationSystem, subComponent);
            packageName = packageName.substring(0, packageName.lastIndexOf("." + name));
            String fullName = Names.getQualifiedName(packageName, name);
            subComponent.setPackageName(packageName);
            subComponent.setFullName(fullName);
            subComponent.addOrderOutput(1);
        }
    }

    private static String getSubComponentName(EMAEquationSystem equationSystem, EMAComponentInstanceSymbol subComponent) {
        String parentName = NameHelper.getPackageOfFullQualifiedName(equationSystem.getName());
        String fullName = NameHelper.calculatePartialName(subComponent.getFullName(), parentName);
        return Joiners.DOT.join(equationSystem.getName(), fullName);
    }

    public static VariableType getTypeOfSubComponent(EMAComponentInstanceSymbol subComponent) {
        String fullName = subComponent.getFullName();
        VariableType type = new VariableType();
        type.setTypeNameMontiCar(fullName);
        type.setTypeNameTargetLanguage(GeneralHelperMethods.getTargetLanguageComponentName(fullName));
        type.setIncludeName(type.getTypeNameTargetLanguage());
        TypeConverter.addNonPrimitiveVariableType(type);
        return type;
    }
}
