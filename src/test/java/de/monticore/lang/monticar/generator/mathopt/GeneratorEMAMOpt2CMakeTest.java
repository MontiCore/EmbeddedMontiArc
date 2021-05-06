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
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Test class for GeneratorEMAMOpt2CMake
 */
public class GeneratorEMAMOpt2CMakeTest extends AbstractSymtabTest {

    protected static List<File> doGenerateModel(String fullModelName) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/");
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullModelName, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorCPP generator = new GeneratorCPP();
        generator.setGenerationTargetPath("./target/generated-sources-cpp/mathopt/cmake/" + fullModelName.substring(fullModelName.lastIndexOf(".") + 1, fullModelName.length()) );

        //TODO adding solver for opt! and uncomment test
//        for (GeneratorImpl g : generator.getGeneratorImpls()) {
//            GeneratorEMAMOpt2CPP gen = (GeneratorEMAMOpt2CPP) g;
//            gen.setGenerateCMake(true);
//            gen.setGenerateTests(true);
//            gen.setCheckModelDir(true);
//            gen.setModelsDirPath(Paths.get("src/test/resources"));
//        }
        generator.useArmadilloBackend();
        generator.setGenerateCMake(true);
        generator.setGenerateTests(true);
        generator.setCheckModelDir(true);
        generator.setModelsDirPath(Paths.get("src/test/resources/mathopt"));

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
