package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
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

        genericComp.getPortsList().forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol("/name1", "std_msgs/Float64MultiArray")));

        List<File> files = generatorRosCpp.generateFiles(genericComp, taggingResolver);

        testFilesAreEqual(files, "basicGenericInstance/", generationTargetPath);
    }

    @Test
    public void testInstanceArrayComp() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/testInstanceArrayComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);
        generatorRosCpp.setGenerateCMake(true);

        ExpandedComponentInstanceSymbol component = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("test.instanceArrayComp.basicPorts[1]", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);


        List<File> files = generatorRosCpp.generateFiles(component, taggingResolver);

        testFilesAreEqual(files, "testInstanceArrayComp/", generationTargetPath);
    }
}
