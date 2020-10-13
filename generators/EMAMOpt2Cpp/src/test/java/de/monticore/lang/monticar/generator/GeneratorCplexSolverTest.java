/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMOpt2CPPSymbolTableHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.Solver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Test class for Cplex solver
 *
 */
public class GeneratorCplexSolverTest {

    /**
     * symbol table as static class variable so it must be created only once
     */
    private static TaggingResolver symtab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver("src/test/resources");

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorEMAMOpt2CPP generator = new GeneratorEMAMOpt2CPP();
        generator.setGenerateCMake(true);
        generator.setPreferedSolver(Solver.Cplex);
        generator.setForceUsePreferredSolver(true);
        generator.setGenerationTargetPath("./target/generated-sources-cmake/cplex/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()) + "/src/");
        return generator.generate(componentInstanceSymbol, symtab);
    }

    @Test
    public void scalarMinimizationCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.scalarMinimizationTest");
    }

    @Test
    public void scalarMaximizationCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.scalarMaximizationTest");
    }

    @Test
    public void transportationProblemCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.transportationProblem");
    }

    @Test
    public void constraintTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.constraintTest");
    }

}
