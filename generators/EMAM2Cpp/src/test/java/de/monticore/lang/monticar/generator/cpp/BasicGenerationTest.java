/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 */
public class BasicGenerationTest extends AbstractSymtabTest {

    @Test
    public void testBasicLookUpInstanceGeneration() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.basicLookUpInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testing/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSampleModel() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.model", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testMathUnitAllLevelsInstancing() throws IOException {
        for (int i = 1; i <= 8; ++i) {
            testMathUnitInstancing(i);
        }
    }


    private void testMathUnitInstancing(int number) throws IOException {
        testMathUnitInstancing(number, false);
        testMathUnitInstancing(number, true);
    }

    private void testMathUnitInstancing(int number, boolean useArmadillo) throws IOException {
        testMathUnitInstancing(number, false, false, useArmadillo);
        testMathUnitInstancing(number, true, false, useArmadillo);
        testMathUnitInstancing(number, false, true, useArmadillo);
        testMathUnitInstancing(number, true, true, useArmadillo);
    }

    private void testMathUnitInstancing(int number, boolean enableThreadingOptimization, boolean enableAlgebraicOptimizations, boolean useArmadillo) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit"+number, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        String additionalPath = "";
        if (enableAlgebraicOptimizations) {
            generatorCPP.setUseAlgebraicOptimizations(true);
            if (enableThreadingOptimization)
                additionalPath = "3";
            else
                additionalPath = "1";
        }
        if (enableThreadingOptimization) {
            generatorCPP.setUseThreadingOptimization(true);
            if (enableAlgebraicOptimizations)
                additionalPath = "3";
            else
                additionalPath = "2";
        }
        if (!enableAlgebraicOptimizations && !enableThreadingOptimization)
            additionalPath = "0";
        String firstPartOfPath = "";
        if (useArmadillo) {
            generatorCPP.useArmadilloBackend();
            firstPartOfPath = "./target/generated-sources-cpp/armadillo/paperMatrixModifier/l";
        } else {
            firstPartOfPath = "./target/generated-sources-cpp/paperMatrixModifier/l";
        }
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath(firstPartOfPath + additionalPath);
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testArrayPortsComp() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.arrayPortsComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testing");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testPortTypeCube() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.portTypeCubeTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testing");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "armadillo/testing/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testClustererNoKMeans() throws IOException{
        testObjectDetectorInstancingL0();
        testObjectDetectorInstancingL1();
        testObjectDetectorInstancingL2();
        testObjectDetectorInstancingL3();
    }

    private void testObjectDetectorInstancingL0() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector4Test" , EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setUseThreadingOptimization(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetectorNoKMeans"  + "/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "detectionObjectDetectorNoKMeans"  + "/l0/";
        //testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector4Test" , EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetectorNoKMeans"  + "/l1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "detectionObjectDetectorNoKMeans"  + "/l1/";
        //testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector4Test" , EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetectorNoKMeans"  + "/l2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "detectionObjectDetectorNoKMeans"  + "/l2/";
        //testFilesAreEqual(files, restPath);
    }


    private void testObjectDetectorInstancingL3() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector4Test" , EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetectorNoKMeans"  + "/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "detectionObjectDetectorNoKMeans"  + "/l3/";
        //testFilesAreEqual(files, restPath);
    }
}
