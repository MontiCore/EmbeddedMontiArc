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
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/detectionObjectDetector");
        generatorCPP.generateFiles(symtab, componentSymbol, symtab);
    }
}
