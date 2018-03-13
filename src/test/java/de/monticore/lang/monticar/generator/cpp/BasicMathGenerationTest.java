package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
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
public class BasicMathGenerationTest extends AbstractSymtabTest {

    @Test
    public void testRowVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.rowVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testColumnVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.columnVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testStaticVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.staticMathSectionVariableTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixConstantVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixConstantVariableMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
    @Test
    public void testMatrixAssignmentTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixAssignmentTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixArrayPortAccessTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixArrayPortAccessTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
}
