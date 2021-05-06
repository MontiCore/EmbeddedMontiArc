/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mathopt;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.Solver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Test class for Cplex solver
 *
 */
@Ignore
public class GeneratorCplexSolverTest extends AbstractSymtabTest {

    /**
     * symbol table as static class variable so it must be created only once
     */
//    private static TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/mathopt");

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/mathopt");
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorCPP generator = new GeneratorCPP();

        //TODO adding solver for opt! and uncomment test
//        for (GeneratorImpl gi : generator.getGeneratorImpls()) {
//            if (gi instanceof GeneratorEMAMOpt2CPP) {
//                GeneratorEMAMOpt2CPP g = (GeneratorEMAMOpt2CPP) gi;
//                g.setPreferedSolver(Solver.Cplex);
//                g.setForceUsePreferredSolver(true);
//            }
//        }

        generator.getMathOptSolverConfig().setPreferedSolver(Solver.Cplex);
        generator.getMathOptSolverConfig().setForceUsePreferredSolver(true);

        generator.setGenerationTargetPath("./target/generated-sources-cpp/mathopt/cplex/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()) + "/src/");
        return generator.generateFiles(symtab, componentInstanceSymbol);
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
