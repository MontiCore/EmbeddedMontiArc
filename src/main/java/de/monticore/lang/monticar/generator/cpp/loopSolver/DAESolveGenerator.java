/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;

import java.util.List;

public interface DAESolveGenerator {
    public void handleEquationSystem(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings);
    public void handleRHS(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings);
}
