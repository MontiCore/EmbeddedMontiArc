package de.monticore.lang.monticar.generator.mqtt;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import static org.junit.Assert.assertNotNull;
import java.io.IOException;
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
        
        GeneratorMqtt generatorMqtt = new GeneratorMqtt();
        
        generatorMqtt.generateMqttAdapter(componentInstanceSymbol);
    }

}
