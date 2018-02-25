package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.DummyMiddlewareGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
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
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new StarBridgeGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-emam/basicGeneration/src/");
        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testCMakeGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new CMakeGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-cmake/CMakeGeneration/src/");
        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testMiddlewareGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new MiddlewareGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-cmake/middlewareGenerator/src/");
        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");
        starBridgeGenerator.add(new DummyMiddlewareGenImpl(), "dummy");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testDistributedTargetGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.distComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);


        StarBridgeGenerator starBridgeGenerator = new DistributedTargetGenerator("./target/generated-sources-cmake/distributed/src/");

        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }
}
