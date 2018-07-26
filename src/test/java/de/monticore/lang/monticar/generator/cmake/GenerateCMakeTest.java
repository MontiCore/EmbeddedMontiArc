package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
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
    private static GeneratorCPP generatorCPP;

    @Before
    public void setUpClass() {
        Log.enableFailQuick(false);
        symtab = createSymTabAndTaggingResolver("src/test/resources");
        generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(true);
    }

    @Test
    public void testCMakeGenerationForBasicConstantAssignment() throws IOException {
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicConstantAssignment", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/cmake/test/BasicConstantAssignment");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "cmake/test/BasicConstantAssignment/";
        testCMakeFilesEqual(files, restPath);
    }

    @Test
    public void testCMakeGenerationForModel() throws IOException {
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.model", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/cmake/testing/Model");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "cmake/testing/Model/";
        testCMakeFilesEqual(files, restPath);
    }

    private void testCMakeFilesEqual(List<File> files, String restPath) {
        List<File> srcFiles = new ArrayList<>();
        List<File> findFiles = new ArrayList<>();
        for (File f : files) {
            if (f.getName().startsWith("Find"))
                findFiles.add(f);
            else
                srcFiles.add(f);
        }
        testFilesAreEqual(srcFiles, restPath);
        testFilesAreEqual(findFiles, restPath + "cmake/");
    }

}
