package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
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
        generatorCPP.useStreamTestTestGeneration();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamtest/autopilot/");
        generatorCPP.useArmadilloBackend();
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        String restPath = "streamtest/autopilot";
        testFilesAreEqual(files, restPath);
    }
}
