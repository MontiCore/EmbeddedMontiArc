/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMOpt2CPPSymbolTableHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Some unit test for generating code for a optimization problem.
 *
 */
public class GeneratorEMAMOpt2CPPTest extends BasicGenerationTest {

    /**
     * symbol table as static class variable so it must be created only once
     */
    private static TaggingResolver symtab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver("src/test/resources");

    /**
     * helper method to generate optimization models in CPP code
     */
    protected static List<File> doGenerateOptimizationModel(String modelName) throws IOException {
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(String.format("de.rwth.monticar.optimization.%s", modelName), EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generator = new GeneratorEMAMOpt2CPP();
        generator.setGenerationTargetPath("./target/generated-sources-cpp/testMath/optimizationSolver/" + modelName);
        List<File> files = generator.generateFiles(symtab, componentSymbol, symtab);
        return files;
    }

    /**
     * Simple quadratic problem min x^2-2x+1 s.t. x >= 0
     */
    @Test
    public void testScalarMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMinimizationTest");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * Simple quadratic problem max x^2-2x+1 s.t. x >= 0
     */
    @Test
    public void testScalarMaximizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMaximizationTest");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * example problem, number 71 from the Hock-Schittkowsky test suite
     * min_{x \in \R^4}	 x_1 x_4 (x_1 + x_2 + x_3) + x_3
     * s.t.
     * x_1 x_2 x_3 x_4 \ge 25
     * x_1^2 + x_2^2 + x_3^2 + x_4^2 = 40
     * 1 \leq x_1, x_2, x_3, x_4 \leq 5
     */
    @Test
    public void testStandardIpoptOptimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("hS71");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * instance of the transportation problem (linear)
     * see https://www.gams.com/products/simple-example/
     */
    @Test
    public void testLPOptimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("transportationProblem");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * test loops in conditions
     */
    @Test
    public void testForLoopConditions() throws IOException {
        List<File> files = doGenerateOptimizationModel("forLoopConditionsTest");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * test conditions of form lower bound <= value <= upper bound
     */
    @Test
    public void testBoundedConditions() throws IOException {
        List<File> files = doGenerateOptimizationModel("boundedConditionsTest");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    /**
     * test models which use a already declared optimization variable
     */
    @Test
    public void existingOptimizationVariable() throws IOException {
        List<File> files = doGenerateOptimizationModel("existingOptimizationVariableTest");
        // TODO: create reference solution
        // String restPath = "testMath/optimizationSolver/";
        // testFilesAreEqual(files, restPath);
    }

    @Test
    public void matrixSumMinimization1Test() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixSumMinimizationTest1");
    }

    @Test
    public void matrixSumMinimization2Test() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixSumMinimizationTest2");
    }

    @Test
    public void constMatrixSumMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("constMatrixSumMinimizationTest");
    }

    @Test
    public void colRowMinTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("colRowMinTest");
    }

    @Test
    public void scalarMultMinTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMultMinTest");
    }

    @Test
    public void matrixTransposeMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixTransposeMinimizationTest");
    }

}
