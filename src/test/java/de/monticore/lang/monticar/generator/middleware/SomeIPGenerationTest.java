package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.someip.GeneratorSomeIP;
import de.monticore.lang.monticar.generator.middleware.impls.SomeIPGenImpl;
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
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        assertNotNull(componentInstanceSymbol);

        File directory = new File(OUT_BASE + "addComp/src/tests_a_addComp/someip");
		directory.mkdirs();

        GeneratorSomeIP generatorSomeIP = new GeneratorSomeIP();
        generatorSomeIP.setGenerationTargetPath(OUT_BASE);
        generatorSomeIP.generateSomeIPAdapter(componentInstanceSymbol);

        componentInstanceSymbol.getPortInstance("in1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(1,2,3));


        //Map<EMAPortSymbol, SomeIPConnectionSymbol> someIPConnectionSymbols = new HashMap<>();
        //componentInstanceSymbol.getPortInstanceList().forEach(p -> {
        //    Collection<TagSymbol> tmpTags = taggingResolver.getTags(p, SomeIPConnectionSymbol.KIND);
        //    if (tmpTags.size() == 1) {
        //        someIPConnectionSymbols.put(p, (SomeIPConnectionSymbol) tmpTags.iterator().next());
        //    }
        //});

        //EMAPortInstanceSymbol in1 = componentInstanceSymbol.getPortInstance("in1").orElse(null);
        //assertNotNull(in1);
        //EMAPortInstanceSymbol in2 = componentInstanceSymbol.getPortInstance("in2").orElse(null);
        //assertNotNull(in2);
        //EMAPortInstanceSymbol out1 = componentInstanceSymbol.getPortInstance("out1").orElse(null);
        //assertNotNull(out1);
        //in1.setMiddlewareSymbol(new SomeIPConnectionSymbol(1, 2, 3));
        //in2.setMiddlewareSymbol(new SomeIPConnectionSymbol(4, 5, 6));
        //out1.setMiddlewareSymbol(new SomeIPConnectionSymbol(7, 8, 9));

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath(OUT_BASE + "addComp/src");
        distributedTargetGenerator.add(new CPPGenImpl(TEST_PATH),"cpp");
        distributedTargetGenerator.add(new SomeIPGenImpl(), "someip");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }
}
