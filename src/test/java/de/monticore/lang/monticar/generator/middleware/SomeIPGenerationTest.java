package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPToEmamTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.someip.GeneratorSomeIP;
import de.monticore.lang.monticar.generator.middleware.impls.SomeIPGenImpl;
import de.monticore.lang.monticar.generator.someip.helper.SomeIPTagHelper;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertNotNull;

public class SomeIPGenerationTest extends AbstractSymtabTest {
    private static final String TEST_PATH = "src/test/resources/";
    private static final String OUT_BASE = "./target/generated-sources-someip/";

    @Test
    public void testSomeIPGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.addComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        SomeIPToEmamTagSchema.registerTagTypes(taggingResolver);

        assertNotNull(componentInstanceSymbol);
        
        SomeIPTagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath(OUT_BASE + "addComp/src");
        distributedTargetGenerator.add(new CPPGenImpl(TEST_PATH),"cpp");
        distributedTargetGenerator.add(new SomeIPGenImpl(), "someip");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }
}
