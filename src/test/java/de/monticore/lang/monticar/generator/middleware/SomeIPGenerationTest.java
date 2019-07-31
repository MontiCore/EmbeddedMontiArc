package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.SomeIPGenImpl;
import de.monticore.lang.monticar.generator.someip.GeneratorSomeIP;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class SomeIPGenerationTest extends AbstractSymtabTest {
    private static final String TEST_PATH = "src/test/resources/";
    private static final String OUT_BASE = "./target/generated-sources-someip/";

    @Test
    public void testSomeIPGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.addComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        assertNotNull(componentInstanceSymbol);

        GeneratorSomeIP generatorSomeIP = new GeneratorSomeIP();

        componentInstanceSymbol.getPortInstance("in1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(1,2,3));
        componentInstanceSymbol.getPortInstance("in2").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(4,5,6));
        componentInstanceSymbol.getPortInstance("out1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(7,8,9));

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath(OUT_BASE + "addComp/src");
        distributedTargetGenerator.add(new CPPGenImpl(TEST_PATH),"cpp");
        distributedTargetGenerator.add(new SomeIPGenImpl(), "someip");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }
}
