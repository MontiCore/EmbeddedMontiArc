package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
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
public class BasicGenerationTest extends AbstractSymtabTest {

    @Test
    public void testBasicLookUpInstanceGeneration() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.basicLookUpInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testing/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testSampleModel() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.model", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
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

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit"+number, ExpandedComponentInstanceSymbol.KIND).orElse(null);
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
        generatorCPP.setGenerationTargetPath(firstPartOfPath + additionalPath);
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testArrayPortsComp() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.arrayPortsComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testing");
        generatorCPP.generateFiles(componentSymbol,symtab);
    }
}
