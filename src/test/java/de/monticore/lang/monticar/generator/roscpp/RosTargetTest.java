package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class RosTargetTest extends AbstractSymtabTest {

    @Test
    public void echoCompTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/echoCompRos/");

        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/echo.yaml", symtab, generatorRosCpp);
        testFilesAreEqual(files, "echo/");

    }

}
