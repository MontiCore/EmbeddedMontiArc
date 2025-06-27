/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
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
public class MathOptimizerTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testRightMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleMatrixRightMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testLeftMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleMatrixLeftMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testComplexLeftMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexMatrixLeftMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexLeftMultiplicationAdditionRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexMatrixLeftMultiplicationAddition2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testComplexRightMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexMatrixRightMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testComplexRightMultiplicationAdditionRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexMatrixRightMultiplicationAddition2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testLeftMultiplicationAdditionMatrixElementRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleMatrixAccessLeftMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testRightMultiplicationAdditionMatrixElementRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleMatrixAccessRightMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    //Multiplication Optimization Tests

    @Test
    public void testSimpleMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleMatrixMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleVectorMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleVectorMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexVectorMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexVectorMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexVectorMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexVectorMultiplication2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleVectorMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleVectorMultiplication2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testSimpleScalarMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleScalarMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexScalarMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexScalarMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleScalarMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.simpleScalarMultiplication2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexChainedMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.complexChainedMultiplication1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testChainedMultiplicationAddition1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("optimizer.chainedMultiplicationAddition", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "optimizer/l1/";
        //testFilesAreEqual(files, restPath);

    }

    @Test
    public void testMatrixModifierRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.matrixModifier", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testMathUnitRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Ignore
    @Test
    public void testMathAssignmentOptimization1Octave() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.normalizedLaplacianInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testMathAssignmentOptimization1Armadillo() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.normalizedLaplacianInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        //generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/optimizer/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }
}
