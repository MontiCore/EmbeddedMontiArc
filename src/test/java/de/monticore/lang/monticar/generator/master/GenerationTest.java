package de.monticore.lang.monticar.generator.master;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

public class GenerationTest extends AbstractSymtabTest {

    @Test
    public void testBasicGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MasterGenerator masterGenerator = new MasterGenerator();
        masterGenerator.add(new CPPImpl(), "cpp");
        masterGenerator.add(new RosCppImpl(), "roscpp");

        masterGenerator.generate(componentInstanceSymbol, taggingResolver, "./target/generated-sources/basicGeneration/");
    }

    @Test
    public void testCMakeGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MasterGenerator masterGenerator = new CMakeMasterGenerator();
        masterGenerator.add(new CPPImpl(), "cpp");
        masterGenerator.add(new RosCppImpl(), "roscpp");

        masterGenerator.generate(componentInstanceSymbol, taggingResolver, "./target/generated-sources/CMakeGeneration/");
    }

    @Test
    public void testMiddlewareMasterGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MasterGenerator masterGenerator = new MiddlewareMasterGenerator();
        masterGenerator.add(new CPPImpl(), "cpp");
        masterGenerator.add(new RosCppImpl(), "roscpp");
        masterGenerator.add(new DummyMiddlewareGenerator(), "dummy");

        masterGenerator.generate(componentInstanceSymbol, taggingResolver, "./target/generated-sources/middlewareMasterGenerator/src/");


    }
}
