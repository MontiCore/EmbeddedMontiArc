/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.someip;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.junit.Test;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class AdapterGenerationTest extends AbstractSymtabTest {

	@Test
    public void testAdapter() throws IOException {

        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");

        // Ros schema is used for now
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        // Create component instance and run the generator
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorSomeIP generatorSomeIP = new GeneratorSomeIP();

        generatorSomeIP.setGenerationTargetPath("./target/generated-sources/");

        // Connect component's ports to topics
        componentInstanceSymbol.getPortInstance("in1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(11,12,13/*,14,15*/));
        componentInstanceSymbol.getPortInstance("in2").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(21,22,23/*,24,25*/));
        componentInstanceSymbol.getPortInstance("out1").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(111,112,113/*,114,115*/));
        componentInstanceSymbol.getPortInstance("out2").orElse(null).setMiddlewareSymbol(new SomeIPConnectionSymbol(121,122,123/*,124,125*/));

        List<File> files = generatorSomeIP.generateSomeIPAdapter(componentInstanceSymbol);

        testFilesAreEqual(files, "echoAdapter/");
    }
}
