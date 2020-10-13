/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.EMAMOpt2CPPSymbolTableHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Test class for GeneratorEMAMOpt2CMake
 */
public class GeneratorEMAMOpt2CMakeTest extends BasicGenerationTest {

    /**
     * symbol table as static class variable so it must be created only once
     */
    private static TaggingResolver symtab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver("src/test/resources");

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorEMAMOpt2CPP generator = new GeneratorEMAMOpt2CPP();
        generator.setGenerationTargetPath("./target/generated-sources-cmake/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()) + "/src/");
        generator.setGenerateCMake(true);
        generator.setGenerateTests(true);
        generator.setCheckModelDir(true);
        generator.setModelsDirPath(Paths.get("src/test/resources"));
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
    public void hs71TestCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.hS71");
    }

    @Test
    public void transportationProblemCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.transportationProblem");
    }

    @Test
    public void boundedConditionsCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.boundedConditionsTest");
    }

    @Test
    public void forLoopConditionsCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.forLoopConditionsTest");
    }

    @Test
    public void existingOptimizationVariableCMakeTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.existingOptimizationVariableTest");
    }

    @Test
    public void matrixSumMinimization1Test() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.matrixSumMinimizationTest1");
    }

    @Test
    public void matrixSumMinimization2Test() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.matrixSumMinimizationTest2");
    }

    @Test
    public void constMatrixSumMinimizationTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.constMatrixSumMinimizationTest");
    }

    @Test
    public void colRowMinTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.colRowMinTest");
    }

    @Test
    public void scalarMultMinTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.scalarMultMinTest");
    }

    @Test
    public void matrixTransposeMinimizationTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.matrixTransposeMinimizationTest");
    }

    @Test
    public void constraintTest() throws IOException {
        List<File> files = doGenerateModel("de.rwth.monticar.optimization.constraintTest");
    }

}
