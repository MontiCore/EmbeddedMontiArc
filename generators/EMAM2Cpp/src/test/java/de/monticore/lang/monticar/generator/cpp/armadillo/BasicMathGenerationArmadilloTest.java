/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.armadillo;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 */
public class BasicMathGenerationArmadilloTest extends AbstractSymtabTest {

    @Test
    public void testRowVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.rowVectorMathSectionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testColumnVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.columnVectorMathSectionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testStaticVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.staticMathSectionVariableTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testMatrixConstantVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.matrixConstantVariableMathSectionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void transposeTest1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.transposeTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void armadilloIndexTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.armadilloIndexTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void sampleComponentTest() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.custom.sampleComponentInst", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/test/custom/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/test/custom/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void samplePacmanMain() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.custom.pacmanSampleGame", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/test/custom/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/test/custom/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void elementwiseMultTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.elementwiseMultTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void vectorColonExpressionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.vectorColonExpressionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void matScalarAddTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.matScalarAddTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
}
