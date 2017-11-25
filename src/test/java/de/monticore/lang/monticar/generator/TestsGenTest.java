package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class TestsGenTest extends AbstractSymtabTest {

    private static final Path MODELS_DIR_PATH = Paths.get("src/test/resources");

    @Test
    public void testMySuperAwesomeComponent1() throws IOException {
        Scope symtab = createSymTab(MODELS_DIR_PATH.toString());
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve(
                "testing.subpackage1.mySuperAwesomeComponent1",
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(MODELS_DIR_PATH);
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MySuperAwesomeComponent1/");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        assertEquals(15, files.size());
    }
}
