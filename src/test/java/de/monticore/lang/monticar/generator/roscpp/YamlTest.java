package de.monticore.lang.monticar.generator.roscpp;

import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertTrue;

public class YamlTest extends AbstractSymtabTest {

    @Test
    public void parseYamlTest() throws IOException {
        TagReader<RosTag> reader = new TagReader<>();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/testRos/");

        List<RosTag> tags = reader.readYAML("src/test/resources/config/config.yaml");
        assertTrue(tags.size() == 1);

        YamlHelper.rosTagToDataHelper(symtab, tags.get(0));

        generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

    }

    @Test
    public void arrayPortTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/arrayGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        YamlHelper.generateFromFile("src/test/resources/config/array.yaml", symtab, generatorRosCpp);

    }

    @Test
    public void multipleComponentsTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/multipleGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        YamlHelper.generateFromFile("src/test/resources/config/multipleComponents.yaml", symtab, generatorRosCpp);

    }
}
