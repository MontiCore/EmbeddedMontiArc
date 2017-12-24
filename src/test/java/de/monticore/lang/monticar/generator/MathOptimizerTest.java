package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
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
public class MathOptimizerTest extends AbstractSymtabTest {

    @Test
    public void testRightMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleMatrixRightMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testLeftMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleMatrixLeftMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testComplexLeftMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexMatrixLeftMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexLeftMultiplicationAdditionRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexMatrixLeftMultiplicationAddition2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testComplexRightMultiplicationAdditionRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexMatrixRightMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testComplexRightMultiplicationAdditionRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexMatrixRightMultiplicationAddition2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testLeftMultiplicationAdditionMatrixElementRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleMatrixAccessLeftMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testRightMultiplicationAdditionMatrixElementRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleMatrixAccessRightMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    //Multiplication Optimization Tests

    @Test
    public void testSimpleMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleMatrixMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleVectorMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleVectorMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexVectorMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexVectorMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexVectorMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexVectorMultiplication2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleVectorMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleVectorMultiplication2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testSimpleScalarMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleScalarMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexScalarMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexScalarMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSimpleScalarMultiplicationRewrite2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.simpleScalarMultiplication2", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testComplexChainedMultiplicationRewrite1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.complexChainedMultiplication1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        testFilesAreEqual(files, restPath);

    }

    @Test
    public void testChainedMultiplicationAddition1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("optimizer.chainedMultiplicationAddition", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "optimizer/l1/";
        //testFilesAreEqual(files, restPath);

    }

    @Test
    public void testMatrixModifierRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.matrixModifier", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l1");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testMathUnitRewrite() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l1");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testMathAssignmentOptimization1() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.normalizedLaplacianInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/optimizer/l1");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }
}
