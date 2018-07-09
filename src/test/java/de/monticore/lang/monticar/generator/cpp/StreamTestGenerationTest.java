package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha Schneiders
 */
public class StreamTestGenerationTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testStreamTestAutopilotTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("de.rwth.armin.modeling.autopilot.autopilot", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1");
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/autopilot/");
        generatorCPP.useArmadilloBackend();
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "streamtest/autopilot";
        //testFilesAreEqual(files, restPath); generated values are random
    }

    @Test
    public void testStreamTestAutopilotAllComponentsTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/autopilot");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1");
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/autopilot/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/autopilot";
        //testFilesAreEqual(files, restPath); generated values are random
    }

    @Test
    public void testStreamTestPacmanControllerSimpleTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/pacman");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("de.rwth.pacman.pacManControllerSimple", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1",2);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/pacman/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "streamtest/pacman";
        //testFilesAreEqual(files, restPath); generated values are random
    }

    //Create image test manually, as generation for these large matrices takes a lot of time
    @Ignore
    @Test
    public void testStreamTestClustererAllComponentsTestGen() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/emastudio/clustering");

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration("1");
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/cluster/");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        List<File> files = generatorCPP.generateFiles(symtab, null, symtab);
        String restPath = "streamtest/cluster";
        //testFilesAreEqual(files, restPath); generated values are random
    }
}
