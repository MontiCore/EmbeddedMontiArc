/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class TestsGenTest extends AbstractSymtabTest {

    private static final Path MODELS_DIR_PATH = Paths.get("src/test/resources/testgentest");

    @Test
    public void testMySuperAwesomeComponent1() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver(MODELS_DIR_PATH.toString());
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subpackage1.mySuperAwesomeComponent1",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(MODELS_DIR_PATH);
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MySuperAwesomeComponent1/");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol, symTab));

//        assertEquals(18, files.size());
        assertEquals(13, files.size()); // TODO: check if 14 is correct here?
    }
}
