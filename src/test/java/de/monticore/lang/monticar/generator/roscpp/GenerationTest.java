package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class GenerationTest extends AbstractSymtabTest {

    @Test
    public void testBasicGenericInstance() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/basicGenericInstance/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        ExpandedComponentInstanceSymbol component = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("test.basicGenericInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        ExpandedComponentInstanceSymbol genericComp = component.getSubComponent("basicGeneric").orElse(null);
        assertNotNull(genericComp);

        genericComp.getPorts().forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol("/name1", "std_msgs/Float64MultiArray")));

        List<File> files = generatorRosCpp.generateFiles(genericComp, taggingResolver);

        testFilesAreEqual(files, "basicGenericInstance/", generationTargetPath);
    }
}
