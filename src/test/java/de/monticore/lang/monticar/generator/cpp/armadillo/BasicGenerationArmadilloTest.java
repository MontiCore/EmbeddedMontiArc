package de.monticore.lang.monticar.generator.cpp.armadillo;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha Schneiders
 */
public class BasicGenerationArmadilloTest extends AbstractSymtabTest {


    @Test
    public void testBasicConstantAssignment() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicConstantAssignment", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testConstantAssignment");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testConstantAssignment/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testBasicConstantAssignment2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicConstantAssignment2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testConstantAssignment2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testConstantAssignment2/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/paperMatrixModifier/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitAlgebraicOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/paperMatrixModifier/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitThreadingOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/paperMatrixModifier/l2/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitBothOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/paperMatrixModifier/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/paperMatrixModifier/l3/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testObjectDetectorInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector/l0");
        generatorCPP.generateFiles(symtab, componentSymbol, symtab);
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

    private void testObjectDetectorInstancingL0(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector" + number, ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l0/";
        testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL1(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector" + number, ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l1/";
        testFilesAreEqual(files, restPath);
    }

    private void testObjectDetectorInstancingL2(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector" + number, ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l2/";
        testFilesAreEqual(files, restPath);
    }


    private void testObjectDetectorInstancingL3(int number) throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector" + number, ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector" + number + "/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/detectionObjectDetector" + number + "/l3/";
        testFilesAreEqual(files, restPath);
    }
}
