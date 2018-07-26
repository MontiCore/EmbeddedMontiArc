package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.YamlHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class RosTargetTest extends AbstractSymtabTest {

    @Test
    public void echoCompTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/echoCompRos/");

        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/echo.yaml", symtab, generatorRosCpp);
        testFilesAreEqual(files, "echo/");

    }

    @Test
    public void intersectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/intersection/");

//        GeneratorCPP generatorCPP = new GeneratorCPP();
//        generatorCPP.setGenerationTargetPath("./target/generated-sources-roscpp/intersection/");
//        generatorCPP.useArmadilloBackend();
//

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("ba.intersection.intersectionController", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(component);

        //List<File> files = generatorCPP.generateFiles(component, symtab);

        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/intersection.yaml", symtab, generatorRosCpp);


    }
}
