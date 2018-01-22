package de.monticore.lang.monticar.generator.roscpp;

import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class yamlTest extends AbstractSymtabTest {

    @Test
    public void parseYamlTest() throws IOException {
        TagReader<RosTag> reader = new TagReader<>();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/testRos/");

        List<RosTag> tags = reader.readYAML("src/test/resources/tests/config/config.yaml");
        assertTrue(tags.size() == 1);

        YamlHelper.rosTagToDataHelper(symtab, tags.get(0));

        generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

    }

    @Test
    public void arrayPortTest() throws IOException {
        TagReader<RosTag> reader = new TagReader<>();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicGenericArrayInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        PortSymbol p1 = componentInstanceSymbol.getPort("val1[1]").orElse(null);
        PortSymbol p2 = componentInstanceSymbol.getPort("val1[2]").orElse(null);

        assertNotNull(p1);
        assertNotNull(p2);


        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/arrayGenCpp/");

        List<RosTag> tags = reader.readYAML("src/test/resources/tests/config/array.yaml");
        assertTrue(tags.size() == 1);

        YamlHelper.rosTagToDataHelper(symtab, tags.get(0));

        generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);


    }
}
