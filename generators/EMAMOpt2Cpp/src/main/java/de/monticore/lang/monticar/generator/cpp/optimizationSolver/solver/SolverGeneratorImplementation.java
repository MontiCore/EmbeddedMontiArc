/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.Problem;

import java.util.List;

/**
 * Implementation of the bridge pattern. Allows dynamic interchangeable implementations of solver generators
 *
 */
public interface SolverGeneratorImplementation {

    public abstract String generateSolverCode(Problem optimizationProblem, List<FileContent> auxillaryFiles, BluePrintCPP bluePrint);

    public abstract List<String> getNecessaryIncludes();
}
