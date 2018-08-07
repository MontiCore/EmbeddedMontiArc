package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * Tests the generation of cmake files
 *
 * @author Christoph Richter
 */
public class GenerateCMakeTest extends AbstractSymtabTest {

    private static TaggingResolver symtab;

    @Before
    public void setUpClass() {
        Log.enableFailQuick(false);
        symtab = createSymTabAndTaggingResolver("src/test/resources");
    }

    @Test
    public void testCMakeGenerationForBasicConstantAssignment() throws IOException {
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicConstantAssignment", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/cmake/test/BasicConstantAssignment");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "cmake/test/BasicConstantAssignment/";
        testCMakeFilesEqual(files, restPath);
    }

    @Test
    public void testCMakeGenerationForModel() throws IOException {
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.model", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/cmake/testing/Model");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "cmake/testing/Model/";
        testCMakeFilesEqual(files, restPath);
    }

    @Test
    public void testCMakeStreamTestGenerationForBasicPortsMath() throws IOException {
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPortsMath", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/cmake/test/BasicPortsMath");
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setCheckModelDir(true);
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "cmake/test/BasicPortsMath/";
        testCMakeFilesEqual(files, restPath);
    }

    private void testCMakeFilesEqual(List<File> files, String restPath) {
        List<File> srcFiles = new ArrayList<>();
        List<File> findFiles = new ArrayList<>();
        List<File> testFiles = new ArrayList<>();
        for (File f : files) {
            if (f.getName().startsWith("Find"))
                findFiles.add(f);
            else if (f.getName().endsWith(".hpp") || f.getName().endsWith(".cpp"))
                testFiles.add(f);
            else if (f.toPath().getParent().endsWith("reporting")) {
                //don't care about reporting files
            }
            else
                srcFiles.add(f);
        }
        testFilesAreEqual(srcFiles, restPath);
        testFilesAreEqual(findFiles, restPath + "cmake/");
        testFilesAreEqual(testFiles, restPath + "test/");
    }

}
