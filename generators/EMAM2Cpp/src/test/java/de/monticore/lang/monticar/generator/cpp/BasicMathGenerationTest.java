/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 */
public class BasicMathGenerationTest extends AbstractSymtabTest {

    @Test
    public void testRowVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.rowVectorMathSectionTest",
                EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testColumnVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.columnVectorMathSectionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testStaticVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.staticMathSectionVariableTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixConstantVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.matrixConstantVariableMathSectionTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
    @Test
    public void testMatrixAssignmentTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.matrixAssignmentTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixArrayPortAccessTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.matrixArrayPortAccessTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testTypeDimensionsTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.typeDimensionsTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
}
