/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mathopt;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Some unit test for generating code for a optimization problem.
 *
 */
public class GeneratorEMAMOpt2CPPTest extends AbstractSymtabTest {


    /**
     * helper method to generate optimization models in CPP code
     */

    protected static List<File> getIpoptGeneratedFiles(File folder){
        List<File> files = new LinkedList<>();
        File dirContents[] = folder.listFiles();
        for (File f : dirContents){
            if(f.getName().contains("CallIpopt")) {
                files.add(f);
            }
        }
        return files;
    }

    protected static List<File> doGenerateOptimizationModel(String modelName) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/");
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(String.format("de.rwth.monticar.optimization.%s", modelName), EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generator = new GeneratorCPP();
        generator.setGenerationTargetPath("./target/generated-sources-cpp/mathopt/generator/" + modelName);
        List<File> files = generator.generateFiles(symtab, componentSymbol);
        File containingDir;
        if(!files.isEmpty()) {
            containingDir = files.get(0).getParentFile();
            files.addAll(getIpoptGeneratedFiles(containingDir));
        }
        return files;
    }

    protected static List<File> doGenerateMathOptModel(String modelName) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/mathopt");
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(String.format("de.rwth.monticar.%s", modelName), EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generator = new GeneratorCPP();
        generator.setGenerationTargetPath("./target/generated-sources-cpp/mathopt/generator/" + modelName);
        List<File> files = generator.generateFiles(symtab, componentSymbol);

        File containingDir;
        if(!files.isEmpty()) {
            containingDir = files.get(0).getParentFile();
            files.addAll(getIpoptGeneratedFiles(containingDir));
        }
        return files;
    }



    @Test
    public void testMPCImplementation() throws IOException {

        List<File> files = doGenerateMathOptModel("mpcautopilot.torcsWrapper");
        String restPath = "mathopt/MPCAutopilot/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testQuadraticOptImplementation() throws IOException {
        List<File> files = doGenerateOptimizationModel("quadraticOpt");
        String restPath = "mathopt/QuadraticOpt/";
        testFilesAreEqual(files, restPath);
    }

    /**
     * Simple quadratic problem min x^2-2x+1 s.t. x >= 0
     */
    @Test
    public void testScalarMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMinimizationTest");
        String restPath = "mathopt/ScalarMinimization/";
        testFilesAreEqual(files, restPath);
    }

    /**
     * Simple quadratic problem max x^2-2x+1 s.t. x >= 0
     */
    @Test
    public void testScalarMaximizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMaximizationTest");
        String restPath = "mathopt/ScalarMaximization/";
        testFilesAreEqual(files, restPath);
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
        String restPath = "mathopt/HS71/";
        testFilesAreEqual(files, restPath);
    }

    /**
     * instance of the transportation problem (linear)
     * see https://www.gams.com/products/simple-example/
     */
    @Test
    public void testLPOptimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("transportationProblem");
        String restPath = "mathopt/TransportationProblem/";
        testFilesAreEqual(files, restPath);
    }

    /**
     * test conditions of form lower bound <= value <= upper bound
     */
    @Test
    public void testBoundedConditions() throws IOException {
        List<File> files = doGenerateOptimizationModel("boundedConditionsTest");
        String restPath = "mathopt/BoundedConditions/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void matrixSumMinimization1Test() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixSumMinimizationTest1");
        String restPath = "mathopt/MatrixSumMinimization1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void matrixSumMinimization2Test() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixSumMinimizationTest2");
        String restPath = "mathopt/MatrixSumMinimization2/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void constMatrixSumMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("constMatrixSumMinimizationTest");
        String restPath = "mathopt/ConstMatrixSumMinimization/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void colRowMinTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("colRowMinTest");
        String restPath = "mathopt/ColRowMin/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void scalarMultMinTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("scalarMultMinTest");
        String restPath = "mathopt/ScalarMultMin/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void matrixTransposeMinimizationTest() throws IOException {
        List<File> files = doGenerateOptimizationModel("matrixTransposeMinimizationTest");
        String restPath = "mathopt/MatrixTransposeMinimization/";
        testFilesAreEqual(files, restPath);
    }

}
