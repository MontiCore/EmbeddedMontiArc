package de.monticore.lang.monticar.generator.someip;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.junit.Test;

public class PrettyPrintGenerationTest extends AbstractSymtabTest {

	@Test
    public void testPrettyPrint() throws IOException {

        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");

        // Ros schema is used for now
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        // Create component instance and run the generator
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorSomeIP generatorSomeIP = new GeneratorSomeIP();

        generatorSomeIP.setGenerationTargetPath("./target/generated-sources/");

        // Connect component's ports to topics
        componentInstanceSymbol.getPortInstance("in1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(1,2,3));

        List<File> files = generatorSomeIP.generatePrettyPrint(componentInstanceSymbol);

        testFilesAreEqual(files, "echo/");
    }

}
