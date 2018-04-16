package de.monticore.lang.monticar.generator.cpp.armadillo;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
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
 * @author Sascha Schneiders
 */
public class BasicMathGenerationArmadilloTest extends AbstractSymtabTest {

    @Test
    public void testRowVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.rowVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testColumnVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.columnVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testStaticVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.staticMathSectionVariableTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testMatrixConstantVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixConstantVariableMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void transposeTest1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.transposeTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void armadilloIndexTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.armadilloIndexTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
}
