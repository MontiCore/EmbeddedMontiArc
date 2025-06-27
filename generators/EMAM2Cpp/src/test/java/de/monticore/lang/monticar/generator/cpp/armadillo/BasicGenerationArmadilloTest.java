/*
 *  (c) https://github.com/MontiCore/monticore
 */

/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.armadillo;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 */
public class BasicGenerationArmadilloTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testBasicConstantAssignment() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicConstantAssignment", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testConstantAssignment");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testConstantAssignment/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testBasicConstantAssignment2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicConstantAssignment2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testConstantAssignment2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testConstantAssignment2/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/paperMatrixModifier/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testMathUnitAlgebraicOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/paperMatrixModifier/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Ignore
    @Test
    public void testMathUnitThreadingOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/paperMatrixModifier/l2/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testMathUnitBothOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/paperMatrixModifier/l3/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testObjectDetectorInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector/l0");
        generatorCPP.generateFiles(symtab, componentSymbol);
    }

    /**
     * Without KMeans for comparision of other operations with libraries that do not support it
     *
     * @throws IOException
     */
    @Test
    public void testObjectDetectorTestInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector4Test", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetectorTest/l0");
        generatorCPP.generateFiles(symtab, componentSymbol);

        generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(false);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetectorTest/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);

        generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetectorTest/l2");
        generatorCPP.generateFiles(symtab, componentSymbol);

        generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetectorTest/l3");
        generatorCPP.generateFiles(symtab, componentSymbol);
    }

    @Test
    public void testAllObjectDetectorInstances() throws IOException {
        for (int i = 1; i <= 9; ++i) {
            testObjectDetectorInstancingL0(i);
            testObjectDetectorInstancingL1(i);
            testObjectDetectorInstancingL2(i);
            testObjectDetectorInstancingL3(i);
        }
    }

    @Test
    public void testAllObjectDetector4Instances() throws Exception {
        testObjectDetectorInstancingL0(4);
        testObjectDetectorInstancingL1(4);
        testObjectDetectorInstancingL2(4);
        testObjectDetectorInstancingL3(4);

    }


    private void testObjectDetectorInstancingL0(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector" + number, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setUseThreadingOptimization(false);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l0/";
        testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL1(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector" + number, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(false);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l1/";
        testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL2(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector" + number, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l2/";
        testFilesAreEqual(files, restPath);
    }


    private void testObjectDetectorInstancingL3(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector" + number, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l3/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testPortInMatrixDefinition() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.portInMatrixDefinition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/portInMatrixDefinition");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/portInMatrixDefinition/";
        testFilesAreEqual(files, restPath);
    }
}
